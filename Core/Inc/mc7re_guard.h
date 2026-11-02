#ifndef MC7RE_GUARD_H
#define MC7RE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

/* Bench RF-link inference ONLY. MC7RE can keep emitting SBUS frames with
 * nominal flags after the radio is powered off. This inference is NOT a
 * substitute for a receiver-provided loss-of-RF signal; flight ARM MUST remain
 * locked until an independent link failsafe is validated. */
#define MC7RE_PRESET_HOLD_MS      300u
#define MC7RE_RECOVERY_HOLD_MS   2000u
#define MC7RE_PRESET_TOL_RAW        2u
#define MC7RE_PRESET_THR_MAX      80u
#define MC7RE_NEUTRAL_TOL_RAW      25u

/* reason: 0 normal; 1 no fresh frame; 2 SBUS frame-lost; 3 SBUS failsafe;
 *         4 inferred preset pattern; 5 boot: no independently proven RF link. */
typedef struct {
    bool latched;
    bool candidate;
    bool candidate_tracking;
    bool recovery_tracking;
    uint32_t candidate_since_ms;
    uint32_t recovery_since_ms;
    uint8_t reason;
} MC7RE_Guard;

static inline bool mc7re_near(uint16_t x, uint16_t target, uint16_t tolerance)
{
    return (x >= target) ? ((uint32_t)x - target <= tolerance)
                         : ((uint32_t)target - x <= tolerance);
}

static inline bool mc7re_fallback_triplet(uint16_t roll, uint16_t pitch,
                                          uint16_t yaw, uint16_t thr)
{
    return mc7re_near(roll, 1000u, MC7RE_PRESET_TOL_RAW) &&
           mc7re_near(pitch, 1000u, MC7RE_PRESET_TOL_RAW) &&
           mc7re_near(yaw, 1000u, MC7RE_PRESET_TOL_RAW) &&
           (thr <= MC7RE_PRESET_THR_MAX);
}

static inline void mc7re_guard_init(MC7RE_Guard *g)
{
    g->latched = true; /* power-on is not proof of RF availability */
    g->candidate = false;
    g->candidate_tracking = false;
    g->recovery_tracking = false;
    g->candidate_since_ms = 0u;
    g->recovery_since_ms = 0u;
    g->reason = 5u;
}

/* raw_reason: 0 -> fresh SBUS without 0x04/0x08, 1/2/3 -> explicit fault.
 * recovery_permitted MUST require transmitter's actual calibrated neutral,
 * ARM OFF, KILL ON, throttle LOW simultaneously. No auto-rearm. */
static inline void mc7re_guard_update(MC7RE_Guard *g, uint32_t now,
                                       uint8_t raw_reason,
                                       uint16_t roll, uint16_t pitch,
                                       uint16_t yaw, uint16_t thr,
                                       bool recovery_permitted)
{
    const bool raw_ok = (raw_reason == 0u);
    const bool preset = raw_ok && mc7re_fallback_triplet(roll, pitch, yaw, thr);
    g->candidate = preset;

    if (!raw_ok) {
        g->latched = true;
        g->reason = raw_reason;
        g->candidate_tracking = false;
    } else if (preset) {
        if (!g->candidate_tracking) {
            g->candidate_tracking = true;
            g->candidate_since_ms = now;
        }
        if ((uint32_t)(now - g->candidate_since_ms) >= MC7RE_PRESET_HOLD_MS) {
            g->latched = true;
            g->reason = 4u;
        }
    } else {
        g->candidate_tracking = false;
    }

    if (g->latched) {
        if (raw_ok && !preset && recovery_permitted) {
            if (!g->recovery_tracking) {
                g->recovery_tracking = true;
                g->recovery_since_ms = now;
            }
            if ((uint32_t)(now - g->recovery_since_ms) >= MC7RE_RECOVERY_HOLD_MS) {
                g->latched = false;
                g->reason = 0u;
                g->recovery_tracking = false;
            }
        } else {
            g->recovery_tracking = false;
        }
    } else {
        g->recovery_tracking = false;
        g->reason = 0u;
    }
}

#endif /* MC7RE_GUARD_H */
