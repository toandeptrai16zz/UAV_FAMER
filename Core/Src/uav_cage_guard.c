#include "uav_cage_guard.h"
#include <math.h>
#include <stddef.h>
static bool exceeded_duration(bool condition, bool *tracking, uint32_t *since,
                              uint32_t now, uint32_t hold_ms) {
    if (!condition) { *tracking=false; return false; }
    if (!*tracking) { *tracking=true; *since=now; return false; }
    return (uint32_t)(now-*since) >= hold_ms;
}
bool UAVCage_IsBenignStop(const UAVCageGuard *g) {
    return g && g->latched &&
        (g->fault == UAV_CAGE_TIMEOUT || g->fault == UAV_CAGE_PILOT_STOP);
}
bool UAVCage_TryClearBenign(UAVCageGuard *g, uint32_t now_ms, bool safe_for_clear) {
    if (!g) return false;
    if (!UAVCage_IsBenignStop(g) || !safe_for_clear) {
        g->clear_since_ms = 0u;
        return false;
    }
    if (g->clear_since_ms == 0u) {
        g->clear_since_ms = now_ms;
        return false;
    }
    if ((uint32_t)(now_ms - g->clear_since_ms) < UAV_CAGE_SAFE_CLEAR_MS) return false;
    UAVCage_Init(g);
    return true;
}
void UAVCage_Init(UAVCageGuard *g) {
    if (!g) return;
    *g=(UAVCageGuard){0};
}
static void trip(UAVCageGuard *g, UAVCageFault reason) {
    g->latched=true; g->fault=reason;
    g->active=false;
}
bool UAVCage_Update(UAVCageGuard *g, const UAVCageInputs *in) {
    if (!g || !in) return true;
    if (g->latched) return true;
    if (!in->armed) {
        /* A disarm ends the current sample window, but after a powered run
         * the test is one-shot to prevent rapid repeat attempts. */
        if (g->active) trip(g, in->pilot_stopped ? UAV_CAGE_PILOT_STOP : UAV_CAGE_INPUT_FAULT);
        return g->latched;
    }
    if (!in->input_ok || !isfinite(in->yaw_dps) || !isfinite(in->roll_dps) ||
        !isfinite(in->pitch_dps) || !isfinite(in->roll_deg) || !isfinite(in->pitch_deg)) {
        trip(g,UAV_CAGE_INPUT_FAULT); return true;
    }
    if (!in->spinning) return false;
    if (!g->active) { g->active=true; g->start_ms=in->now_ms; }
    if ((uint32_t)(in->now_ms-g->start_ms) >= UAV_CAGE_MAX_ACTIVE_MS) {
        trip(g,UAV_CAGE_TIMEOUT); return true;
    }
    if (fabsf(in->yaw_dps) >= UAV_CAGE_RATE_HARD_DPS ||
        fabsf(in->roll_dps) >= UAV_CAGE_RATE_HARD_DPS ||
        fabsf(in->pitch_dps) >= UAV_CAGE_RATE_HARD_DPS) {
        trip(g,UAV_CAGE_RATE_HARD); return true;
    }
    if (fabsf(in->roll_deg)>=UAV_CAGE_ANGLE_HARD_DEG ||
        fabsf(in->pitch_deg)>=UAV_CAGE_ANGLE_HARD_DEG) {
        trip(g,UAV_CAGE_ATTITUDE); return true;
    }
    if (exceeded_duration(fabsf(in->yaw_dps)>=UAV_CAGE_YAW_SOFT_DPS,
                          &g->yaw_timing,&g->yaw_start_ms,in->now_ms,UAV_CAGE_SOFT_HOLD_MS)) {
        trip(g,UAV_CAGE_FAST_YAW); return true;
    }
    if (exceeded_duration(fabsf(in->roll_deg)>=UAV_CAGE_ANGLE_SOFT_DEG ||
                          fabsf(in->pitch_deg)>=UAV_CAGE_ANGLE_SOFT_DEG,
                          &g->tilt_timing,&g->tilt_start_ms,in->now_ms,UAV_CAGE_SOFT_HOLD_MS)) {
        trip(g,UAV_CAGE_ATTITUDE); return true;
    }
    return false;
}
