#ifndef UAV_TELEMETRY_H
#define UAV_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define UAV_TELEMETRY_PACKET_SIZE  88u
#define UAV_TELEMETRY_VERSION      1u
#define UAV_TELEMETRY_TYPE_FLIGHT  1u

/* flags bit definitions; all multi-byte wire fields are little-endian. */
#define UAV_TELEM_FLAG_LINK_OK       (1u << 0)
#define UAV_TELEM_FLAG_FAILSAFE      (1u << 1)
#define UAV_TELEM_FLAG_ARMED         (1u << 2)
#define UAV_TELEM_FLAG_SPINNING      (1u << 3)
#define UAV_TELEM_FLAG_IMU_FRESH     (1u << 4)
#define UAV_TELEM_FLAG_KILL_ACTIVE   (1u << 5)
#define UAV_TELEM_FLAG_ALT_HOLD      (1u << 6)
#define UAV_TELEM_FLAG_IMU_CAL       (1u << 7)
#define UAV_TELEM_FLAG_RANGE_VALID   (1u << 8)
#define UAV_TELEM_FLAG_DMA_ERROR     (1u << 9)
#define UAV_TELEM_FLAG_OUTPUT_ACTIVE (1u << 10)
#define UAV_TELEM_FLAG_CONFIG_READY  (1u << 11)

/* A snapshot only. This module never controls arming, PID, mixer or DShot. */
typedef struct {
    uint32_t timestamp_ms;
    uint16_t flags;

    uint16_t rc_throttle;
    uint16_t rc_roll;
    uint16_t rc_pitch;
    uint16_t rc_yaw;

    int16_t roll_cdeg;
    int16_t pitch_cdeg;

    int16_t gyro_roll_dps_x10;
    int16_t gyro_pitch_dps_x10;
    int16_t gyro_yaw_dps_x10;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t pid_roll_x10;
    int16_t pid_pitch_x10;
    int16_t pid_yaw_x10;

    uint16_t base_dshot;
    uint16_t mixer_scale_x1000;
    int16_t mixer_offset;

    int16_t mix_x10[4];
    uint16_t requested_dshot[4];
    uint16_t output_dshot[4];

    /* 0xFFFF means not measured / not connected. Do not fabricate values. */
    uint16_t battery_mv;
    uint16_t current_ma;
    uint16_t range_cm;

    uint16_t imu_dt_max_ms;
    uint16_t motor_dt_max_ms;
    uint16_t dshot_errors;
    uint16_t rx_age_ms;
} UAVTelemetrySample;

void UAVTelemetry_Reset(void);
bool UAVTelemetry_Send(UART_HandleTypeDef *uart, const UAVTelemetrySample *sample);
void UAVTelemetry_OnTxComplete(UART_HandleTypeDef *uart);
/* Returns true only when an in-flight telemetry TX was aborted by this error. */
bool UAVTelemetry_OnTxError(UART_HandleTypeDef *uart);
uint32_t UAVTelemetry_GetDropped(void);
uint32_t UAVTelemetry_GetCompleted(void);

#endif /* UAV_TELEMETRY_H */
