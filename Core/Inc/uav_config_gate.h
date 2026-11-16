#ifndef UAV_CONFIG_GATE_H
#define UAV_CONFIG_GATE_H

#include <stdbool.h>
#include <stdint.h>

/* Independent RC safety gate for six-channel PID configuration.
 * Does not arm the vehicle, start motors or change PID gains.
 * Keeping this pure and HAL-free allows host-side unit testing. */
#define UAV_CONFIG_SAFE_HOLD_MS 2000u

typedef struct {
    bool link_stable;
    bool rf_guard_ok;
    bool config_switch_on; /* always true in 6CH mode, real switch in 8CH mode */
    bool kill_on;
    bool arm_on;
    bool throttle_low;
    bool armed;
    bool spinning;
    bool bench_in_progress;
} UAVConfigGateInputs;

typedef struct {
    uint32_t safe_since_ms;
    bool tracking;
} UAVConfigGateState;

static inline void UAVConfigGate_Reset(UAVConfigGateState *s)
{
    if (s) { s->safe_since_ms = 0u; s->tracking = false; }
}

static inline bool UAVConfigGate_Update(UAVConfigGateState *s, uint32_t now_ms,
                                        const UAVConfigGateInputs *v)
{
    if (!s || !v) return false;
    const bool safe = v->link_stable && v->rf_guard_ok &&
                      v->config_switch_on && v->kill_on && !v->arm_on &&
                      v->throttle_low && !v->armed && !v->spinning &&
                      !v->bench_in_progress;
    if (!safe) { UAVConfigGate_Reset(s); return false; }
    if (!s->tracking) { s->tracking = true; s->safe_since_ms = now_ms; return false; }
    return (uint32_t)(now_ms - s->safe_since_ms) >= UAV_CONFIG_SAFE_HOLD_MS;
}

#endif
