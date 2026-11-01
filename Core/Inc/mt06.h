#ifndef MT06_H
#define MT06_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT06_UART_BAUDRATE          115200u
#define MT06_UART_RX_BUFFER_SIZE    64u
#define MT06_MAX_PACKET_SIZE        280u

#define MT06_MIN_RANGE_CM           2u
#define MT06_MAX_RANGE_CM           600u
#define MT06_DEFAULT_ORIENTATION_DOWN 25u /* MAV_SENSOR_ROTATION_PITCH_270 */
#define MT06_DATA_TIMEOUT_MS        150u

typedef struct {
    UART_HandleTypeDef *huart;

    uint8_t rx_buffer[MT06_UART_RX_BUFFER_SIZE];
    uint8_t packet[MT06_MAX_PACKET_SIZE];
    uint16_t packet_length;
    uint16_t expected_length;

    volatile uint16_t distance_cm;
    volatile uint16_t min_distance_cm;
    volatile uint16_t max_distance_cm;
    volatile uint8_t sensor_type;
    volatile uint8_t sensor_id;
    volatile uint8_t orientation;
    volatile uint8_t covariance_cm2;
    volatile uint32_t last_update_ms;
    volatile uint32_t frames_ok;
    volatile uint32_t frames_bad;
    volatile uint32_t uart_errors;

    volatile bool sample_received;
    volatile bool rx_rearm_pending;
    bool initialized;
} MT06_HandleTypeDef;

typedef struct {
    uint16_t distance_cm;
    uint16_t min_distance_cm;
    uint16_t max_distance_cm;
    uint8_t sensor_type;
    uint8_t sensor_id;
    uint8_t orientation;
    uint8_t covariance_cm2;
    uint32_t age_ms;
    uint32_t last_update_ms;
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t uart_errors;
    bool initialized;
    bool sample_received;
    bool fresh;
    bool orientation_down;
    bool valid;
} MT06_Data;

/* Sensor must be configured for UART / MAVLink at 115200 baud. */
bool MT06_Init(MT06_HandleTypeDef *dev, UART_HandleTypeDef *huart);

/* Call this from the main loop; it rearms Receive-to-Idle after RX callbacks. */
void MT06_Update(MT06_HandleTypeDef *dev);

/* Forward these HAL callbacks from the application's existing UART callbacks. */
void MT06_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void MT06_ErrorCallback(UART_HandleTypeDef *huart);

bool MT06_GetData(const MT06_HandleTypeDef *dev, MT06_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* MT06_H */
