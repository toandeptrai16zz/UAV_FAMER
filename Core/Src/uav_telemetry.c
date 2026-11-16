#include "uav_telemetry.h"
#include <stddef.h>
#include <string.h>

#define UAV_TELEM_SYNC_0 0xA5u
#define UAV_TELEM_SYNC_1 0x5Au

static uint8_t tx_packet[UAV_TELEMETRY_PACKET_SIZE];
static UART_HandleTypeDef *tx_uart;
static volatile uint8_t tx_inflight;
static uint16_t tx_sequence;
static uint16_t bb_tx_sequence;
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
    bb_tx_sequence = 0u;
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

/* Type 2, exactly 88 bytes. The old 88-byte type 1 flight protocol is UNCHANGED.
 * Offsets: 12 capture_id, 14 total, 16 index, 18 reserved, 20..73 sample,
 * 72..85 reserved, 86..87 CRC16. Never transmit while UART is busy. */
_Static_assert(sizeof(UAVRateSample) == 56u, "Unexpected blackbox sample alignment");
bool UAVTelemetry_SendBlackbox(UART_HandleTypeDef *uart, const UAVRateSample *s,
                               uint16_t index, uint16_t total, uint16_t capture_id)
{
    if (!uart || !s) return false;
    if (tx_inflight || uart->gState != HAL_UART_STATE_READY) return false;
    (void)memset(tx_packet, 0, sizeof(tx_packet));
    tx_packet[0] = UAV_TELEM_SYNC_0;
    tx_packet[1] = UAV_TELEM_SYNC_1;
    tx_packet[2] = UAV_TELEMETRY_VERSION;
    tx_packet[3] = UAV_BB_PACKET_TYPE;
    /* Independent counter: buffered packets must not appear as missing live
     * flight frames in the existing Ground Station sequence-gap statistics. */
    put_u16_le(&tx_packet[4], bb_tx_sequence++);
    put_u32_le(&tx_packet[6], s->stamp_ms);
    put_u16_le(&tx_packet[12], capture_id);
    put_u16_le(&tx_packet[14], total);
    put_u16_le(&tx_packet[16], index);
    uint16_t p = 20u;
    put_u32_le(&tx_packet[p], s->stamp_ms); p += 4u;
    put_u16_le(&tx_packet[p], s->pid_dt_us); p += 2u;
    put_u16_le(&tx_packet[p], s->motor_dt_us); p += 2u;
    for (unsigned axis = 0; axis < 3u; axis++) { put_i16_le(&tx_packet[p], s->gyro_x10[axis]); p += 2u; }
    for (unsigned axis = 0; axis < 3u; axis++) { put_i16_le(&tx_packet[p], s->set_x10[axis]); p += 2u; }
    for (unsigned axis = 0; axis < 3u; axis++) { put_i16_le(&tx_packet[p], s->p_x10[axis]); p += 2u; }
    for (unsigned axis = 0; axis < 3u; axis++) { put_i16_le(&tx_packet[p], s->i_x10[axis]); p += 2u; }
    for (unsigned axis = 0; axis < 3u; axis++) { put_i16_le(&tx_packet[p], s->d_x10[axis]); p += 2u; }
    for (unsigned j = 0; j < 4u; j++) { put_u16_le(&tx_packet[p], s->motor[j]); p += 2u; }
    put_u16_le(&tx_packet[p], s->base); p += 2u;
    put_u16_le(&tx_packet[p], s->mixer_scale_x1000); p += 2u;
    put_u16_le(&tx_packet[p], s->imu_seq_lsb); p += 2u;
    put_u16_le(&tx_packet[p], s->flags); p += 2u;
    if (p != 74u) return false;
    put_u16_le(&tx_packet[86], crc16_ccitt(tx_packet, 86u));
    tx_uart = uart;
    tx_inflight = 1u;
    if (HAL_UART_Transmit_IT(uart, tx_packet, UAV_TELEMETRY_PACKET_SIZE) != HAL_OK) {
        tx_inflight = 0u;
        tx_dropped++;
        return false;
    }
    return true;
}

/* Type 3 R5.5: same 88-byte transport/CRC. No changes to V1/type1 and
 * V1/type2 packets. Offsets 12..19: capture_id, total, index, kind (all u16).
 * Offset 20: HAL tick_ms (u32); offset 24: DWT CYCCNT (u32).
 * Offset 28..39: raw accel XYZ and gyro XYZ, int16 big-signed decoded LE.
 * Offset 40: HCLK MHz (u16). CRC at 86. */
bool UAVTelemetry_SendIMURaw(UART_HandleTypeDef *uart, const UAVIMURawSample *s,
                             uint16_t index, uint16_t total, uint16_t capture_id,
                             uint16_t kind, uint16_t hclk_mhz)
{
    if (!uart || !s || tx_inflight || uart->gState != HAL_UART_STATE_READY) return false;
    (void)memset(tx_packet, 0, sizeof(tx_packet));
    tx_packet[0] = UAV_TELEM_SYNC_0;
    tx_packet[1] = UAV_TELEM_SYNC_1;
    tx_packet[2] = UAV_TELEMETRY_VERSION;
    tx_packet[3] = UAV_IMU_RAW_PACKET_TYPE;
    put_u16_le(&tx_packet[4], bb_tx_sequence++);
    put_u32_le(&tx_packet[6], s->tick_ms);
    put_u16_le(&tx_packet[12], capture_id);
    put_u16_le(&tx_packet[14], total);
    put_u16_le(&tx_packet[16], index);
    put_u16_le(&tx_packet[18], kind);
    put_u32_le(&tx_packet[20], s->tick_ms);
    put_u32_le(&tx_packet[24], s->cycle_count);
    for (unsigned i = 0u; i < 3u; ++i) {
        put_i16_le(&tx_packet[28u + 2u * i], s->accel[i]);
        put_i16_le(&tx_packet[34u + 2u * i], s->gyro[i]);
    }
    put_u16_le(&tx_packet[40], hclk_mhz);
    put_u16_le(&tx_packet[86], crc16_ccitt(tx_packet, 86u));
    tx_uart = uart;
    tx_inflight = 1u;
    if (HAL_UART_Transmit_IT(uart, tx_packet, UAV_TELEMETRY_PACKET_SIZE) != HAL_OK) {
        tx_inflight = 0u;
        tx_dropped++;
        return false;
    }
    return true;
}
