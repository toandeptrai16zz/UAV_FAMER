/* Bench-only two-loop controller. All gains are illustrative UNTUNED values.
 * Do NOT unlock motors based solely on model/simulation tuning. */
#include "flight_pid.h"
#include <math.h>
#include <stddef.h>

#define ANGLE_KP             4.0f     /* deg/s per deg angle error */
#define MAX_RATE_DPS         80.0f
#define RATE_KP_ROLL         1.5f     /* DShot counts / (deg/s) */
#define RATE_KP_PITCH        1.5f
#define RATE_KP_YAW          1.0f
#define RATE_KI              0.0f     /* Await identified plant; no integral windup */
#define RATE_KD              0.0f     /* Await gyro noise / sample-rate validation */
#define INTEGRAL_LIMIT      30.0f
#define CORRECTION_LIMIT  200.0f

static float limit(float x, float bound) {
    return x > bound ? bound : (x < -bound ? -bound : x);
}

void FlightPID_Reset(FlightPIDState *s) {
    if (!s) return;
    for (unsigned i=0; i<3; ++i) {
        s->integrator[i]=0.0f;
        s->gyro_prev[i]=0.0f;
        s->gyro_deriv_filt[i]=0.0f;
    }
    s->has_prev=false;
}

bool FlightPID_Step(FlightPIDState *s, const FlightPIDInput *in, FlightPIDOutput *out) {
    if (!s || !in || !out) return false;
    const float v[] = {in->roll_set_deg, in->pitch_set_deg, in->yaw_rate_set_dps,
                       in->roll_deg, in->pitch_deg, in->gyro_roll_dps,
                       in->gyro_pitch_dps, in->gyro_yaw_dps, in->dt_s};
    for (unsigned i=0; i<sizeof(v)/sizeof(v[0]); ++i) if (!isfinite(v[i])) {
        FlightPID_Reset(s); return false;
    }
    if (in->dt_s <= 0.0f || in->dt_s > 0.020f) { FlightPID_Reset(s); return false; }
    const float gyro[] = {in->gyro_roll_dps, in->gyro_pitch_dps, in->gyro_yaw_dps};
    const float target[] = {
        limit(ANGLE_KP*(in->roll_set_deg-in->roll_deg),MAX_RATE_DPS),
        limit(ANGLE_KP*(in->pitch_set_deg-in->pitch_deg),MAX_RATE_DPS),
        limit(in->yaw_rate_set_dps,MAX_RATE_DPS)
    };
    const float kp[] = {RATE_KP_ROLL,RATE_KP_PITCH,RATE_KP_YAW};
    float u[3];
    for (unsigned i=0;i<3;++i) {
        float error=target[i]-gyro[i];
        float derivative=0.0f;
        if (s->has_prev) derivative=(gyro[i]-s->gyro_prev[i])/in->dt_s;
        s->gyro_deriv_filt[i] = 0.2f*derivative + 0.8f*s->gyro_deriv_filt[i];
        float candidate_i = limit(s->integrator[i]+error*in->dt_s,INTEGRAL_LIMIT);
        float unconstrained=kp[i]*error+RATE_KI*candidate_i-RATE_KD*s->gyro_deriv_filt[i];
        if (in->allow_integrator && fabsf(unconstrained) < CORRECTION_LIMIT) {
            s->integrator[i] = candidate_i;
        } else if (!in->allow_integrator) s->integrator[i] = 0.0f;
        u[i]=limit(kp[i]*error+RATE_KI*s->integrator[i]-RATE_KD*s->gyro_deriv_filt[i],CORRECTION_LIMIT);
        s->gyro_prev[i]=gyro[i];
    }
    s->has_prev=true;
    out->roll=u[0]; out->pitch=u[1]; out->yaw=u[2];
    return true;
}

void FlightPID_MixX(float roll, float pitch, float yaw, float c[4]) {
    if (!c) return;
    /* Verified geometric placement (not verified IMU-axis signs):
     * M1 front-left CCW, M2 front-right CW,
     * M3 rear-left CW, M4 rear-right CCW.
     * Coordinates assume positive roll = right-side-down,
     * positive pitch = nose-up, positive yaw = clockwise from above.
     * A positive axis correction commands the corresponding +torque;
     * mount orientation must be validated with the passive tilt test.
     */
    c[0] = +roll +pitch +yaw; /* M1: front-left  CCW */
    c[1] = -roll +pitch -yaw; /* M2: front-right CW  */
    c[2] = +roll -pitch -yaw; /* M3: rear-left   CW  */
    c[3] = -roll -pitch +yaw; /* M4: rear-right  CCW */
}

/* The DShot command loop and IMU sampling are not guaranteed to coincide.
 * A PID result must be held for intervening motor-update ticks, rather
 * than replaced with a zero correction on ticks without a new IMU sample. */
void FlightPIDSampleHold_Reset(FlightPIDSampleHold *hold) {
    if (!hold) return;
    hold->output.roll = 0.0f;
    hold->output.pitch = 0.0f;
    hold->output.yaw = 0.0f;
    hold->updated_ms = 0u;
    hold->valid = false;
}

bool FlightPIDSampleHold_Store(FlightPIDSampleHold *hold, const FlightPIDOutput *output, uint32_t now_ms) {
    if (!hold || !output) return false;
    if (!isfinite(output->roll) || !isfinite(output->pitch) || !isfinite(output->yaw)) {
        FlightPIDSampleHold_Reset(hold);
        return false;
    }
    hold->output = *output;
    hold->updated_ms = now_ms;
    hold->valid = true;
    return true;
}

bool FlightPIDSampleHold_Read(const FlightPIDSampleHold *hold, uint32_t now_ms, uint32_t max_age_ms, FlightPIDOutput *output) {
    if (!hold || !output || !hold->valid || ((uint32_t)(now_ms - hold->updated_ms) > max_age_ms)) return false;
    *output = hold->output;
    return true;
}
