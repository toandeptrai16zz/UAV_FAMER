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
/* X layout assumes M1=front-right, M2=rear-right, M3=rear-left,
   M4=front-left and spin signs +,-,+,-. MUST verify against real wiring.
   Return values are corrections in DShot counts, NOT motor thrust. */
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
#endif
