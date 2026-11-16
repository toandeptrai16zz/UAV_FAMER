#ifndef UAV_IMU_RAW_H
#define UAV_IMU_RAW_H
#include <stdbool.h>
#include <stdint.h>

/* R5.5 diagnostic-only raw IMU ring. One producer/consumer in the main loop.
 * Sampled once per successful MPU6500 SPI read (TARGET 1 kHz, not guaranteed).
 * Does not influence the PID, motor outputs, filters or ARM logic. */
#define UAV_IMU_RAW_SAMPLES 1024u
#define UAV_IMU_RAW_PACKET_TYPE 3u
#define UAV_IMU_RAW_CAPTURE_BOOT 1u
#define UAV_IMU_RAW_CAPTURE_MOTOR 2u

typedef struct {
    uint32_t tick_ms;
    uint32_t cycle_count;
    int16_t accel[3];
    int16_t gyro[3];
} UAVIMURawSample;

void UAVIMURaw_Reset(void);
void UAVIMURaw_Push(const UAVIMURawSample *sample);
void UAVIMURaw_MotorStarted(void);
void UAVIMURaw_MotorStopped(void);
bool UAVIMURaw_Peek(UAVIMURawSample *sample, uint16_t *index,
                    uint16_t *total, uint16_t *capture_id, uint16_t *kind);
void UAVIMURaw_Sent(void);
bool UAVIMURaw_Dumping(void);
uint16_t UAVIMURaw_Total(void);
#endif
