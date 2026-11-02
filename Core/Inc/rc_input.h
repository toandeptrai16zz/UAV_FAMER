#ifndef RC_INPUT_H
#define RC_INPUT_H
#include <stdint.h>
#include <stdbool.h>

/* Measured in user SBUS logs. Calibrate again if transmitter settings change. */
#define RC_THR_MIN_RAW       40u
#define RC_THR_MAX_RAW     1640u
#define RC_ROLL_CENTER      961u
#define RC_PITCH_CENTER     992u
#define RC_YAW_CENTER      1099u
#define RC_STICK_MIN        172u
#define RC_STICK_MAX       1811u
#define RC_STICK_DEADBAND    20u
#define RC_LOW_PERCENT        5u

typedef struct {
    bool low_seen, high_seen;
    bool low_tracking, high_tracking;
    uint32_t low_since_ms, high_since_ms;
} RCThrottleCalibration;

void RC_CalibrationReset(RCThrottleCalibration *cal);
/* Collect endpoints only when kill switch ON, ARM switch OFF and link valid. */
void RC_CalibrationObserve(RCThrottleCalibration *cal, uint32_t now_ms,
                           bool safe_calibration_position, uint16_t raw);
bool RC_CalibrationComplete(const RCThrottleCalibration *cal);
bool RC_ThrottleRawValid(uint16_t raw);
bool RC_ThrottleIsLow(uint16_t raw);
float RC_StickNormalized(uint16_t raw, uint16_t center);
#endif
