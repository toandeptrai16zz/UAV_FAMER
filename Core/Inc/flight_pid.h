#ifndef FLIGHT_PID_H
#define FLIGHT_PID_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float integrator[3];
    float gyro_prev[3];
    float gyro_deriv_filt[3];
    bool has_prev;
} FlightPIDState;

typedef struct {
    float roll_set_deg, pitch_set_deg, yaw_rate_set_dps;
    float roll_deg, pitch_deg;
    float gyro_roll_dps, gyro_pitch_dps, gyro_yaw_dps;
    float dt_s;
    bool allow_integrator;
} FlightPIDInput;

typedef struct { float roll, pitch, yaw; } FlightPIDOutput;
void FlightPID_Reset(FlightPIDState *state);
bool FlightPID_Step(FlightPIDState *state, const FlightPIDInput *input, FlightPIDOutput *output);
/* User-confirmed quad X map (looking from above, nose forward):
 * M1 front-left  CCW / PA6, M2 front-right CW  / PA7,
 * M3 rear-left   CW  / PB0, M4 rear-right CCW / PB1.
 * Mixer assumes +roll right-side-down, +pitch nose-up, +yaw clockwise.
 * Physical IMU sign and torque response NOT YET VERIFIED.
 * Outputs are relative DShot corrections, not measured thrust. */
/* Retain the most recently calculated PID correction between IMU samples.
   A stale or invalid correction is never returned. Diagnostic only while locked. */
typedef struct {
    FlightPIDOutput output;
    uint32_t updated_ms;
    bool valid;
} FlightPIDSampleHold;
void FlightPIDSampleHold_Reset(FlightPIDSampleHold *hold);
bool FlightPIDSampleHold_Store(FlightPIDSampleHold *hold, const FlightPIDOutput *output, uint32_t now_ms);
bool FlightPIDSampleHold_Read(const FlightPIDSampleHold *hold, uint32_t now_ms, uint32_t max_age_ms, FlightPIDOutput *output);
void FlightPID_MixX(float roll, float pitch, float yaw, float corrections[4]);
/* Preserve differential control authority when collective thrust saturates.
 * Outputs are shifted together into [min_command, max_command]; corrections are
 * scaled only when their span cannot fit in the available command range. */
bool FlightPID_DesaturateMixer(float collective, const float corrections[4],
                               float min_command, float max_command,
                               float outputs[4], float *applied_scale,
                               float *collective_offset);
#endif
