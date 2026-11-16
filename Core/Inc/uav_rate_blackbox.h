#ifndef UAV_RATE_BLACKBOX_H
#define UAV_RATE_BLACKBOX_H
#include <stdbool.h>
#include <stdint.h>
/* R5 200 Hz ring blackbox. No motor commands, heap, interrupts, UART or Wi-Fi in sampler. */
#define UAV_BB_SAMPLES 512u
#define UAV_BB_PERIOD_MS 5u
#define UAV_BB_POST_TRIGGER 160u
#define UAV_BB_PACKET_BYTES 88u
#define UAV_BB_PACKET_TYPE 2u

typedef struct {
    uint32_t stamp_ms;
    uint16_t pid_dt_us;
    uint16_t motor_dt_us;
    int16_t gyro_x10[3];
    int16_t set_x10[3];
    int16_t p_x10[3];
    int16_t i_x10[3];
    int16_t d_x10[3];
    uint16_t motor[4];
    uint16_t base;
    uint16_t mixer_scale_x1000;
    uint16_t imu_seq_lsb;
    uint16_t flags;
} UAVRateSample;

void UAVBB_Reset(void);
/* Triggered by cage motor-motion or by hand-motion in ARM-locked mode. */
void UAVBB_Push(const UAVRateSample *sample, bool trigger);
/* Freeze whatever pre-event data exists when the safety interlock trips.
 * Never waits for post-trigger samples that may stop when IMU becomes invalid.
 * UART upload remains forbidden until disarm/KILL/output zero. */
void UAVBB_FreezeNow(void);
bool UAVBB_Peek(UAVRateSample *sample, uint16_t *index, uint16_t *count, uint16_t *capture_id);
void UAVBB_Sent(void);
void UAVBB_Tick(uint32_t now_ms);
bool UAVBB_Dumping(void);
uint16_t UAVBB_Total(void);
#endif
