#ifndef UAV_CAGE_GUARD_H
#define UAV_CAGE_GUARD_H
#include <stdbool.h>
#include <stdint.h>
/* One short captive test per power cycle. Diagnostic interlock, NOT a
 * replacement for physical power cutout or a shielded test fixture. */
#define UAV_CAGE_MAX_ACTIVE_MS      5500u
#define UAV_CAGE_YAW_SOFT_DPS       110.0f
#define UAV_CAGE_RATE_HARD_DPS      240.0f
#define UAV_CAGE_ANGLE_SOFT_DEG      22.0f
#define UAV_CAGE_ANGLE_HARD_DEG      35.0f
#define UAV_CAGE_SOFT_HOLD_MS        80u
#define UAV_CAGE_SAFE_CLEAR_MS      3000u

typedef enum {
    UAV_CAGE_OK=0, UAV_CAGE_TIMEOUT=1, UAV_CAGE_FAST_YAW=2,
    UAV_CAGE_ATTITUDE=3, UAV_CAGE_RATE_HARD=4,
    UAV_CAGE_INPUT_FAULT=5, UAV_CAGE_PILOT_STOP=6
} UAVCageFault;
typedef struct {
    bool active;
    bool latched;
    uint32_t start_ms;
    uint32_t clear_since_ms;
    uint32_t yaw_start_ms;
    uint32_t tilt_start_ms;
    bool yaw_timing;
    bool tilt_timing;
    UAVCageFault fault;
} UAVCageGuard;
typedef struct {
    uint32_t now_ms;
    bool armed, spinning, input_ok, pilot_stopped;
    float yaw_dps, roll_dps, pitch_dps;
    float roll_deg, pitch_deg;
} UAVCageInputs;
/* Only a benign TIMEOUT or PILOT_STOP can be cleared through a verified,
 * disarmed safe state. Unsafe cutouts require power-cycle and inspection. */
bool UAVCage_IsBenignStop(const UAVCageGuard *g);
/* Clears only benign latches after a *continuous* verified safe hold. */
bool UAVCage_TryClearBenign(UAVCageGuard *g, uint32_t now_ms, bool safe_for_clear);
void UAVCage_Init(UAVCageGuard *g);
bool UAVCage_Update(UAVCageGuard *g, const UAVCageInputs *in);
#endif
