#include "rc_input.h"
#include <stddef.h>

#define RC_ENDPOINT_HOLD_MS   250u
#define RC_LOW_ENDPOINT_MAX   80u
#define RC_HIGH_ENDPOINT_MIN 1610u
#define RC_THR_VALID_MIN      20u
#define RC_THR_VALID_MAX    1811u  /* SBUS high endpoint; throttle map still saturates at RC_THR_MAX_RAW */

void RC_CalibrationReset(RCThrottleCalibration *cal) {
    if (!cal) return;
    cal->low_seen = false; cal->high_seen = false;
    cal->low_tracking = false; cal->high_tracking = false;
    cal->low_since_ms = 0u; cal->high_since_ms = 0u;
}

bool RC_ThrottleRawValid(uint16_t raw) {
    return raw >= RC_THR_VALID_MIN && raw <= RC_THR_VALID_MAX;
}

bool RC_ThrottleIsLow(uint16_t raw) {
    const uint32_t threshold = RC_THR_MIN_RAW +
      ((RC_THR_MAX_RAW - RC_THR_MIN_RAW) * RC_LOW_PERCENT / 100u);
    return RC_ThrottleRawValid(raw) && raw <= threshold;
}

void RC_CalibrationObserve(RCThrottleCalibration *cal, uint32_t now_ms,
                           bool safe, uint16_t raw) {
    if (!cal) return;
    if (!safe || !RC_ThrottleRawValid(raw)) {
        cal->low_tracking = false; cal->high_tracking = false;
        return;
    }
    if (raw <= RC_LOW_ENDPOINT_MAX) {
        if (!cal->low_tracking) {cal->low_since_ms=now_ms;cal->low_tracking=true;}
        if ((uint32_t)(now_ms-cal->low_since_ms) >= RC_ENDPOINT_HOLD_MS) cal->low_seen=true;
    } else cal->low_tracking=false;
    if (raw >= RC_HIGH_ENDPOINT_MIN) {
        if (!cal->high_tracking) {cal->high_since_ms=now_ms;cal->high_tracking=true;}
        if ((uint32_t)(now_ms-cal->high_since_ms) >= RC_ENDPOINT_HOLD_MS) cal->high_seen=true;
    } else cal->high_tracking=false;
}

bool RC_CalibrationComplete(const RCThrottleCalibration *cal) {
    return cal && cal->low_seen && cal->high_seen;
}

float RC_StickNormalized(uint16_t raw, uint16_t center) {
    int32_t d = (int32_t)raw-(int32_t)center;
    if (d <= (int32_t)RC_STICK_DEADBAND && d >= -(int32_t)RC_STICK_DEADBAND) return 0.0f;
    if (d > 0) {
        const float denom = (float)(RC_STICK_MAX-center-RC_STICK_DEADBAND);
        float v = (float)(d-RC_STICK_DEADBAND)/denom;
        return (v>1.0f) ? 1.0f : v;
    }
    const float denom = (float)(center-RC_STICK_MIN-RC_STICK_DEADBAND);
    float v = (float)(d+(int32_t)RC_STICK_DEADBAND)/denom;
    return (v < -1.0f) ? -1.0f : v;
}
