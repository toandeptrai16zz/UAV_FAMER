#ifndef UAV_CONFIG_H
#define UAV_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* UART LPUART1 only. Independent from UDP telemetry v1.
 * Request: C3 3C 01 type seq_lo seq_hi len payload crc_lo crc_hi
 * Reply:   same header; type | 80; payload[0] is status.
 * CRC = CCITT-FALSE over sync..payload. Commands: 10 GET, 11 APPLY,
 * 12 SAVE, 13 ENTER_ROM_BOOTLOADER (bench only, opt-in).
 */
#define UAV_CONFIG_ENABLE_ROM_OTA   0 /* change to 1 only AFTER props-off bench verification */
#define UAV_CONFIG_CMD_GET         0x10u
#define UAV_CONFIG_CMD_APPLY       0x11u
#define UAV_CONFIG_CMD_SAVE        0x12u
#define UAV_CONFIG_CMD_BOOT        0x13u
/* R5.6: explicit reference-pose calibration; never auto-tare at startup.
 * All writes require the existing authoritative CONFIG_READY gate.
 * GET returns status,safe,valid,u32 roll(float),u32 pitch(float).
 * CAPTURE/CLEAR modify RAM only. SAVE writes reference + PID as one journal. */
#define UAV_CONFIG_CMD_LEVEL_GET       0x14u
#define UAV_CONFIG_CMD_LEVEL_CAPTURE   0x15u
#define UAV_CONFIG_CMD_LEVEL_SAVE      0x16u
#define UAV_CONFIG_CMD_LEVEL_CLEAR     0x17u
#define UAV_CONFIG_STATUS_OK       0u
#define UAV_CONFIG_STATUS_LOCKED   1u
#define UAV_CONFIG_STATUS_LENGTH   2u
#define UAV_CONFIG_STATUS_RANGE    3u
#define UAV_CONFIG_STATUS_DISABLED 4u
#define UAV_CONFIG_STATUS_FLASH    5u
#define UAV_CONFIG_STATUS_BUSY     6u

void UAVConfig_Init(UART_HandleTypeDef *uart);
/* Base attitude is independent of pilot reference (IMU->airframe mount trim).
 * Gyro bias calibration stays in the MPU6500 path and is not modified here. */
float UAVConfig_LevelRoll(void);
float UAVConfig_LevelPitch(void);
void UAVConfig_LevelGet(float *roll, float *pitch, bool *valid);
void UAVConfig_LevelObserve(float base_roll, float base_pitch,
                            float rate_roll, float rate_pitch, float rate_yaw,
                            float accel_confidence, bool safe_and_fresh);
void UAVConfig_OnRxComplete(UART_HandleTypeDef *uart);
void UAVConfig_OnTxComplete(UART_HandleTypeDef *uart);
void UAVConfig_OnError(UART_HandleTypeDef *uart);
/* Service in MAIN loop, not in ISR. safe == authoritative CH8+ARM+KILL+low-throttle+disarm gate. */
void UAVConfig_Service(bool safe);
/* Firmware boot only called when ack sent and physical CONFIG mode remains safe. */
void UAVConfig_ServiceBoot(bool safe);
#endif
