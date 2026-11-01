#include "mt06.h"

#include <stddef.h>
#include <string.h>

/*
 * MicoAir MT-06 UART default output is MAVLink at 115200 baud / 50 Hz.
 * This driver consumes MAVLink v1 (0xFE) and v2 (0xFD) DISTANCE_SENSOR
 * message 132 only. CRC_EXTRA for DISTANCE_SENSOR is 85.
 *
 * MAVLink DISTANCE_SENSOR payload (minimum 14 bytes):
 *   0..3  time_boot_ms
 *   4..5  min_distance [cm]
 *   6..7  max_distance [cm]
 *   8..9  current_distance [cm]
 *   10    type
 *   11    id
 *   12    orientation (25 = downward)
 *   13    covariance [cm^2]
 */

#define MAVLINK_STX_V1             0xFEu
#define MAVLINK_STX_V2             0xFDu
#define MAVLINK_MSG_ID_DISTANCE_SENSOR 132u
#define MAVLINK_DISTANCE_SENSOR_CRC_EXTRA 85u
#define MAVLINK_DISTANCE_SENSOR_MIN_LEN 14u
#define MAVLINK_V2_SIGNED_FLAG     0x01u
#define MAVLINK_V2_SIGNATURE_LEN   13u

static MT06_HandleTypeDef *active_mt06;

static uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = (uint8_t)(data ^ (uint8_t)(crc & 0xFFu));
    tmp ^= (uint8_t)(tmp << 4);
    return (uint16_t)((crc >> 8) ^
                      ((uint16_t)tmp << 8) ^
                      ((uint16_t)tmp << 3) ^
                      ((uint16_t)tmp >> 4));
}

static uint16_t mavlink_crc_calculate(const uint8_t *data, uint16_t length,
                                      uint8_t crc_extra)
{
    uint16_t crc = 0xFFFFu;

    for (uint16_t i = 0u; i < length; ++i) {
        crc = mavlink_crc_accumulate(data[i], crc);
    }
    crc = mavlink_crc_accumulate(crc_extra, crc);
    return crc;
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void mt06_publish_distance(MT06_HandleTypeDef *dev,
                                  const uint8_t *payload,
                                  uint8_t payload_length)
{
    uint16_t min_cm;
    uint16_t max_cm;
    uint16_t distance_cm;

    if (payload_length < MAVLINK_DISTANCE_SENSOR_MIN_LEN) {
        dev->frames_bad++;
        return;
    }

    min_cm = read_u16_le(&payload[4]);
    max_cm = read_u16_le(&payload[6]);
    distance_cm = read_u16_le(&payload[8]);

    /* Publish only a complete, CRC-checked MAVLink DISTANCE_SENSOR frame. */
    dev->min_distance_cm = min_cm;
    dev->max_distance_cm = max_cm;
    dev->distance_cm = distance_cm;
    dev->sensor_type = payload[10];
    dev->sensor_id = payload[11];
    dev->orientation = payload[12];
    dev->covariance_cm2 = payload[13];
    dev->last_update_ms = HAL_GetTick();
    dev->sample_received = true;
    dev->frames_ok++;
}

static void mt06_process_packet(MT06_HandleTypeDef *dev)
{
    const uint8_t *packet = dev->packet;
    uint8_t payload_length;
    uint8_t stx;
    uint16_t crc_offset;
    uint16_t crc_input_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint32_t message_id;
    uint16_t payload_offset;

    if ((dev == NULL) || (dev->packet_length < 8u)) {
        return;
    }

    stx = packet[0];
    payload_length = packet[1];

    if (stx == MAVLINK_STX_V1) {
        /* V1: STX, LEN, SEQ, SYSID, COMPID, MSGID, PAYLOAD, CRC */
        if (dev->packet_length != (uint16_t)(payload_length + 8u)) {
            dev->frames_bad++;
            return;
        }
        message_id = packet[5];
        payload_offset = 6u;
        crc_offset = (uint16_t)(6u + payload_length);
        crc_input_length = (uint16_t)(5u + payload_length);
    } else if (stx == MAVLINK_STX_V2) {
        /* V2: STX + 9-byte header, payload, CRC, optional 13-byte signature. */
        uint8_t incompat_flags;
        uint16_t expected;

        if (dev->packet_length < 12u) {
            dev->frames_bad++;
            return;
        }

        incompat_flags = packet[2];
        expected = (uint16_t)(payload_length + 12u);
        if ((incompat_flags & MAVLINK_V2_SIGNED_FLAG) != 0u) {
            expected = (uint16_t)(expected + MAVLINK_V2_SIGNATURE_LEN);
        }
        if (dev->packet_length != expected) {
            dev->frames_bad++;
            return;
        }

        message_id = (uint32_t)packet[7] |
                     ((uint32_t)packet[8] << 8) |
                     ((uint32_t)packet[9] << 16);
        payload_offset = 10u;
        crc_offset = (uint16_t)(10u + payload_length);
        crc_input_length = (uint16_t)(9u + payload_length);
    } else {
        return;
    }

    /* Ignore other MAVLink messages without interpreting their payload. */
    if (message_id != MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        return;
    }

    if (payload_length < MAVLINK_DISTANCE_SENSOR_MIN_LEN) {
        dev->frames_bad++;
        return;
    }

    received_crc = read_u16_le(&packet[crc_offset]);
    calculated_crc = mavlink_crc_calculate(&packet[1],
                                            crc_input_length,
                                            MAVLINK_DISTANCE_SENSOR_CRC_EXTRA);
    if (received_crc != calculated_crc) {
        dev->frames_bad++;
        return;
    }

    mt06_publish_distance(dev, &packet[payload_offset], payload_length);
}

static void mt06_parser_feed(MT06_HandleTypeDef *dev, uint8_t byte)
{
    if (dev->packet_length == 0u) {
        if ((byte == MAVLINK_STX_V1) || (byte == MAVLINK_STX_V2)) {
            dev->packet[0] = byte;
            dev->packet_length = 1u;
            dev->expected_length = 0u;
        }
        return;
    }

    if (dev->packet_length >= MT06_MAX_PACKET_SIZE) {
        dev->packet_length = 0u;
        dev->expected_length = 0u;
        dev->frames_bad++;
        return;
    }

    dev->packet[dev->packet_length++] = byte;

    if (dev->packet_length == 2u) {
        const uint8_t stx = dev->packet[0];
        const uint8_t payload_length = dev->packet[1];

        if (stx == MAVLINK_STX_V1) {
            dev->expected_length = (uint16_t)(payload_length + 8u);
        } else if (stx == MAVLINK_STX_V2) {
            /* Need incompat_flags (byte 2) before signed-frame length is known. */
            dev->expected_length = (uint16_t)(payload_length + 12u);
        }
    }

    if ((dev->packet[0] == MAVLINK_STX_V2) &&
        (dev->packet_length == 3u) &&
        ((dev->packet[2] & MAVLINK_V2_SIGNED_FLAG) != 0u)) {
        dev->expected_length =
            (uint16_t)(dev->expected_length + MAVLINK_V2_SIGNATURE_LEN);
    }

    if ((dev->expected_length != 0u) &&
        (dev->packet_length == dev->expected_length)) {
        mt06_process_packet(dev);
        dev->packet_length = 0u;
        dev->expected_length = 0u;
    }
}

static bool mt06_start_rx(MT06_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->huart == NULL) ||
        (dev->huart->RxState != HAL_UART_STATE_READY)) {
        return false;
    }

    __HAL_UART_CLEAR_FLAG(dev->huart,
                          UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_NEF | UART_CLEAR_OREF);

    status = HAL_UARTEx_ReceiveToIdle_IT(dev->huart,
                                         dev->rx_buffer,
                                         sizeof(dev->rx_buffer));
    if (status == HAL_OK) {
        dev->rx_rearm_pending = false;
        return true;
    }

    dev->uart_errors++;
    dev->rx_rearm_pending = true;
    return false;
}

bool MT06_Init(MT06_HandleTypeDef *dev, UART_HandleTypeDef *huart)
{
    if ((dev == NULL) || (huart == NULL)) {
        return false;
    }

    memset(dev, 0, sizeof(*dev));
    dev->huart = huart;
    dev->min_distance_cm = MT06_MIN_RANGE_CM;
    dev->max_distance_cm = MT06_MAX_RANGE_CM;
    dev->rx_rearm_pending = true;
    active_mt06 = dev;
    dev->initialized = true;

    /* Start receive-to-idle; the main loop retries if UART is not ready yet. */
    (void)mt06_start_rx(dev);
    return dev->initialized;
}

void MT06_Update(MT06_HandleTypeDef *dev)
{
    if ((dev != NULL) && dev->initialized && dev->rx_rearm_pending) {
        (void)mt06_start_rx(dev);
    }
}

void MT06_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    MT06_HandleTypeDef *dev = active_mt06;

    if ((dev == NULL) || !dev->initialized || (huart != dev->huart)) {
        return;
    }

    if (size > sizeof(dev->rx_buffer)) {
        size = sizeof(dev->rx_buffer);
    }

    for (uint16_t i = 0u; i < size; ++i) {
        mt06_parser_feed(dev, dev->rx_buffer[i]);
    }

    dev->rx_rearm_pending = true;
}

void MT06_ErrorCallback(UART_HandleTypeDef *huart)
{
    MT06_HandleTypeDef *dev = active_mt06;

    if ((dev == NULL) || !dev->initialized || (huart != dev->huart)) {
        return;
    }

    dev->uart_errors++;
    dev->packet_length = 0u;
    dev->expected_length = 0u;
    dev->rx_rearm_pending = true;
}

bool MT06_GetData(const MT06_HandleTypeDef *dev, MT06_Data *data)
{
    uint32_t primask;
    uint32_t now;

    if ((dev == NULL) || (data == NULL)) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    data->distance_cm = dev->distance_cm;
    data->min_distance_cm = dev->min_distance_cm;
    data->max_distance_cm = dev->max_distance_cm;
    data->sensor_type = dev->sensor_type;
    data->sensor_id = dev->sensor_id;
    data->orientation = dev->orientation;
    data->covariance_cm2 = dev->covariance_cm2;
    data->last_update_ms = dev->last_update_ms;
    data->frames_ok = dev->frames_ok;
    data->frames_bad = dev->frames_bad;
    data->uart_errors = dev->uart_errors;
    data->sample_received = dev->sample_received;

    if (primask == 0u) {
        __enable_irq();
    }

    now = HAL_GetTick();
    data->age_ms = data->sample_received ? (now - data->last_update_ms) : 0xFFFFFFFFu;
    data->initialized = dev->initialized;
    data->fresh = data->sample_received && (data->age_ms <= MT06_DATA_TIMEOUT_MS);
    data->orientation_down =
        (data->orientation == MT06_DEFAULT_ORIENTATION_DOWN);

    data->valid = data->fresh &&
                  data->orientation_down &&
                  (data->distance_cm >= data->min_distance_cm) &&
                  (data->distance_cm <= data->max_distance_cm) &&
                  (data->distance_cm >= MT06_MIN_RANGE_CM) &&
                  (data->distance_cm <= MT06_MAX_RANGE_CM);

    return data->valid;
}
