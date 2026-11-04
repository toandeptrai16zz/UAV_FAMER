#include "uav_telemetry.h"
#include <stddef.h>
#include <string.h>

#define UAV_TELEM_SYNC_0 0xA5u
#define UAV_TELEM_SYNC_1 0x5Au

static uint8_t tx_packet[UAV_TELEMETRY_PACKET_SIZE];
static UART_HandleTypeDef *tx_uart;
static volatile uint8_t tx_inflight;
static uint16_t tx_sequence;
static volatile uint32_t tx_dropped;
static volatile uint32_t tx_completed;

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8);
}

static void put_i16_le(uint8_t *dst, int16_t value)
{
    put_u16_le(dst, (uint16_t)value);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)(value >> 24);
}

/* CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, xorout=0. */
static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void build_packet(const UAVTelemetrySample *s)
{
    size_t p = 0u;
    tx_packet[p++] = UAV_TELEM_SYNC_0;
    tx_packet[p++] = UAV_TELEM_SYNC_1;
    tx_packet[p++] = UAV_TELEMETRY_VERSION;
    tx_packet[p++] = UAV_TELEMETRY_TYPE_FLIGHT;

    put_u16_le(&tx_packet[p], tx_sequence++); p += 2u;
    put_u32_le(&tx_packet[p], s->timestamp_ms); p += 4u;
    put_u16_le(&tx_packet[p], s->flags); p += 2u;

    put_u16_le(&tx_packet[p], s->rc_throttle); p += 2u;
    put_u16_le(&tx_packet[p], s->rc_roll); p += 2u;
    put_u16_le(&tx_packet[p], s->rc_pitch); p += 2u;
    put_u16_le(&tx_packet[p], s->rc_yaw); p += 2u;

    put_i16_le(&tx_packet[p], s->roll_cdeg); p += 2u;
    put_i16_le(&tx_packet[p], s->pitch_cdeg); p += 2u;

    put_i16_le(&tx_packet[p], s->gyro_roll_dps_x10); p += 2u;
    put_i16_le(&tx_packet[p], s->gyro_pitch_dps_x10); p += 2u;
    put_i16_le(&tx_packet[p], s->gyro_yaw_dps_x10); p += 2u;

    put_i16_le(&tx_packet[p], s->accel_x_raw); p += 2u;
    put_i16_le(&tx_packet[p], s->accel_y_raw); p += 2u;
    put_i16_le(&tx_packet[p], s->accel_z_raw); p += 2u;

    put_i16_le(&tx_packet[p], s->pid_roll_x10); p += 2u;
    put_i16_le(&tx_packet[p], s->pid_pitch_x10); p += 2u;
    put_i16_le(&tx_packet[p], s->pid_yaw_x10); p += 2u;

    put_u16_le(&tx_packet[p], s->base_dshot); p += 2u;
    put_u16_le(&tx_packet[p], s->mixer_scale_x1000); p += 2u;
    put_i16_le(&tx_packet[p], s->mixer_offset); p += 2u;

    for (uint8_t i = 0u; i < 4u; ++i) {
        put_i16_le(&tx_packet[p], s->mix_x10[i]); p += 2u;
    }
    for (uint8_t i = 0u; i < 4u; ++i) {
        put_u16_le(&tx_packet[p], s->requested_dshot[i]); p += 2u;
    }
    for (uint8_t i = 0u; i < 4u; ++i) {
        put_u16_le(&tx_packet[p], s->output_dshot[i]); p += 2u;
    }

    put_u16_le(&tx_packet[p], s->battery_mv); p += 2u;
    put_u16_le(&tx_packet[p], s->current_ma); p += 2u;
    put_u16_le(&tx_packet[p], s->range_cm); p += 2u;
    put_u16_le(&tx_packet[p], s->imu_dt_max_ms); p += 2u;
    put_u16_le(&tx_packet[p], s->motor_dt_max_ms); p += 2u;
    put_u16_le(&tx_packet[p], s->dshot_errors); p += 2u;
    put_u16_le(&tx_packet[p], s->rx_age_ms); p += 2u;

    /* Packet is 88 bytes: CRC occupies the final two bytes (offset 86). */
    if (p == (UAV_TELEMETRY_PACKET_SIZE - 2u)) {
        const uint16_t crc = crc16_ccitt(tx_packet, p);
        put_u16_le(&tx_packet[p], crc);
    }
}

void UAVTelemetry_Reset(void)
{
    tx_uart = NULL;
    tx_inflight = 0u;
    tx_sequence = 0u;
    tx_dropped = 0u;
    tx_completed = 0u;
    (void)memset(tx_packet, 0, sizeof(tx_packet));
}

bool UAVTelemetry_Send(UART_HandleTypeDef *uart, const UAVTelemetrySample *sample)
{
    if ((uart == NULL) || (sample == NULL)) {
        tx_dropped++;
        return false;
    }

    /* Same UART is also used by the legacy ASCII logger. HAL's gState is the
     * arbiter: never overlap DMA/IT transfers and never wait in the motor loop. */
    if ((tx_inflight != 0u) || (uart->gState != HAL_UART_STATE_READY)) {
        tx_sequence++; /* expose missed telemetry slots as sequence gaps */
        tx_dropped++;
        return false;
    }

    tx_uart = uart;
    build_packet(sample);
    tx_inflight = 1u;
    if (HAL_UART_Transmit_IT(uart, tx_packet, (uint16_t)sizeof(tx_packet)) != HAL_OK) {
        tx_inflight = 0u;
        tx_dropped++;
        return false;
    }
    return true;
}

void UAVTelemetry_OnTxComplete(UART_HandleTypeDef *uart)
{
    if ((tx_inflight != 0u) && (uart == tx_uart)) {
        tx_inflight = 0u;
        tx_completed++;
    }
}

bool UAVTelemetry_OnTxError(UART_HandleTypeDef *uart)
{
    if ((tx_inflight != 0u) && (uart == tx_uart)) {
        tx_inflight = 0u;
        tx_dropped++;
        return true;
    }
    return false;
}

uint32_t UAVTelemetry_GetDropped(void)
{
    return tx_dropped;
}

uint32_t UAVTelemetry_GetCompleted(void)
{
    return tx_completed;
}
