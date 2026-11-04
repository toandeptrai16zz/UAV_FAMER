/* Initial cascaded attitude/rate controller for FLIGHT CANDIDATE.
 * ANGLE_KP and RATE_KP are provisional. RATE_KI and RATE_KD intentionally
 * remain zero until real gyro noise, motor response and airframe dynamics are
 * measured. This is not a validated/tuned flight PID. */
#include "flight_pid.h"
#include <math.h>
#include <stddef.h>

/* Deliberately preserve the original flight-candidate defaults. */
static const FlightPIDGains default_gains = {
    .angle_kp = 4.0f, .max_rate_dps = 80.0f,
    .rate_kp = {1.5f, 1.5f, 1.0f},
    .rate_ki = {0.0f, 0.0f, 0.0f},
    .rate_kd = {0.0f, 0.0f, 0.0f},
    .integral_limit = 30.0f, .correction_limit = 200.0f
};
static FlightPIDGains active_gains = {
    .angle_kp = 4.0f, .max_rate_dps = 80.0f,
    .rate_kp = {1.5f, 1.5f, 1.0f},
    .rate_ki = {0.0f, 0.0f, 0.0f},
    .rate_kd = {0.0f, 0.0f, 0.0f},
    .integral_limit = 30.0f, .correction_limit = 200.0f
};

void FlightPID_DefaultGains(FlightPIDGains *g) { if (g) *g = default_gains; }
void FlightPID_GetGains(FlightPIDGains *g) { if (g) *g = active_gains; }

bool FlightPID_ValidateGains(const FlightPIDGains *g) {
    if (!g || !isfinite(g->angle_kp) || g->angle_kp < 0.0f || g->angle_kp > 8.0f ||
        !isfinite(g->max_rate_dps) || g->max_rate_dps < 1.0f || g->max_rate_dps > 150.0f ||
        !isfinite(g->integral_limit) || g->integral_limit < 0.0f || g->integral_limit > 50.0f ||
        !isfinite(g->correction_limit) || g->correction_limit < 1.0f || g->correction_limit > 250.0f) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(g->rate_kp[i]) || g->rate_kp[i] < 0.0f || g->rate_kp[i] > 3.0f ||
            !isfinite(g->rate_ki[i]) || g->rate_ki[i] < 0.0f || g->rate_ki[i] > 1.0f ||
            !isfinite(g->rate_kd[i]) || g->rate_kd[i] < 0.0f || g->rate_kd[i] > 0.1f) return false;
    }
    return true;
}

bool FlightPID_SetGains(const FlightPIDGains *g) {
    if (!FlightPID_ValidateGains(g)) return false;
    active_gains = *g; /* Only main loop when disarmed; no ISR calls this. */
    return true;
}

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
        limit(active_gains.angle_kp*(in->roll_set_deg-in->roll_deg),active_gains.max_rate_dps),
        limit(active_gains.angle_kp*(in->pitch_set_deg-in->pitch_deg),active_gains.max_rate_dps),
        limit(in->yaw_rate_set_dps,active_gains.max_rate_dps)
    };
    const float *kp = active_gains.rate_kp;
    float u[3];
    for (unsigned i=0;i<3;++i) {
        float error=target[i]-gyro[i];
        float derivative=0.0f;
        if (s->has_prev) derivative=(gyro[i]-s->gyro_prev[i])/in->dt_s;
        s->gyro_deriv_filt[i] = 0.2f*derivative + 0.8f*s->gyro_deriv_filt[i];
        float candidate_i = limit(s->integrator[i]+error*in->dt_s,active_gains.integral_limit);
        float unconstrained=kp[i]*error+active_gains.rate_ki[i]*candidate_i-active_gains.rate_kd[i]*s->gyro_deriv_filt[i];
        if (in->allow_integrator && fabsf(unconstrained) < active_gains.correction_limit) {
            s->integrator[i] = candidate_i;
        } else if (!in->allow_integrator) s->integrator[i] = 0.0f;
        u[i]=limit(kp[i]*error+active_gains.rate_ki[i]*s->integrator[i]-active_gains.rate_kd[i]*s->gyro_deriv_filt[i],active_gains.correction_limit);
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

bool FlightPID_DesaturateMixer(float collective, const float corrections[4],
                               float min_command, float max_command,
                               float outputs[4], float *applied_scale,
                               float *collective_offset) {
    if (!corrections || !outputs || !isfinite(collective) ||
        !isfinite(min_command) || !isfinite(max_command) ||
        !(max_command > min_command)) {
        return false;
    }

    float corr_min = corrections[0];
    float corr_max = corrections[0];
    if (!isfinite(corr_min)) return false;

    for (unsigned i = 1; i < 4; ++i) {
        if (!isfinite(corrections[i])) return false;
        if (corrections[i] < corr_min) corr_min = corrections[i];
        if (corrections[i] > corr_max) corr_max = corrections[i];
    }

    const float command_span = max_command - min_command;
    const float correction_span = corr_max - corr_min;
    float scale = 1.0f;
    if (correction_span > command_span && correction_span > 0.0f) {
        scale = command_span / correction_span;
    }

    const float mixed_min = collective + corr_min * scale;
    const float mixed_max = collective + corr_max * scale;
    const float low_shift = min_command - mixed_min;
    const float high_shift = max_command - mixed_max;
    float offset = 0.0f;

    /* The scaled correction span now fits. Shift the entire motor set only
       when needed; do not shrink all PID corrections just because collective
       thrust is near one output rail. */
    if (low_shift > 0.0f) {
        offset = low_shift;
    } else if (high_shift < 0.0f) {
        offset = high_shift;
    }

    for (unsigned i = 0; i < 4; ++i) {
        float value = collective + corrections[i] * scale + offset;
        if (!isfinite(value)) return false;
        if (value < min_command) value = min_command;
        if (value > max_command) value = max_command;
        outputs[i] = value;
    }

    if (applied_scale) *applied_scale = scale;
    if (collective_offset) *collective_offset = offset;
    return true;
}

