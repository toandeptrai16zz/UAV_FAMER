/* Cascaded angle -> rate controller. 2026-11-05 lab/cage trial profile.
 * The observed 11+ deg sustained roll error needs motor/axis verification;
 * controller changes alone cannot establish stability. The small I term is
 * updated only while NEAR LEVEL, neutral sticks and with real motor output,
 * and frozen during transients to avoid winding against cage/tethers.
 * D remains OFF until high-rate gyro noise is logged. */
#include "flight_pid.h"
#include <math.h>
#include <stddef.h>

/* Distinct controlled experiment: preserve verified Angle P=3.0, Rate P,
 * add SMALL bounded I (max 6 / 6 / 3 DShot) only within near-level gate.
 * Saved Flash gains loaded by UAVConfig_Init take precedence. */
static const FlightPIDGains default_gains = {
    .angle_kp = 3.0f, .max_rate_dps = 80.0f,
    .rate_kp = {1.5f, 1.5f, 1.0f},
    .rate_ki = {0.20f, 0.20f, 0.10f},
    .rate_kd = {0.0f, 0.0f, 0.0f},
    .integral_limit = 30.0f, .correction_limit = 200.0f
};
static FlightPIDGains active_gains = {
    .angle_kp = 3.0f, .max_rate_dps = 80.0f,
    .rate_kp = {1.5f, 1.5f, 1.0f},
    .rate_ki = {0.20f, 0.20f, 0.10f},
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

/* Recalculate D-only low-pass per real dt (not fixed alpha per iteration).
 * A first-order 25 Hz cutoff limits differentiator noise. D is OFF by
 * default; this ensures future D tuning does not depend on timing jitter. */
#define PID_DTERM_LPF_HZ 25.0f
#define PID_TAU_TWO_PI 6.283185307179586f

static float limit(float x, float bound) {
    return x > bound ? bound : (x < -bound ? -bound : x);
}

void FlightPID_Reset(FlightPIDState *s) {
    if (!s) return;
    for (unsigned i=0; i<3; ++i) {
        s->integrator[i]=0.0f;
        s->gyro_prev[i]=0.0f;
        s->gyro_deriv_filt[i]=0.0f;
        s->rate_set_dps[i]=0.0f;
        s->rate_error_dps[i]=0.0f;
        s->pid_p[i]=0.0f;
        s->pid_i[i]=0.0f;
        s->pid_d[i]=0.0f;
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
    /* Angle loop sets EULER angle rates; gyro measures BODY angular rates.
     * With yaw motion and tilted airframe they are not interchangeable.
     * Convert requested Euler roll/pitch rates to body p/q using the
     * measured yaw rate for attitude-hold decoupling.  The same 3-2-1
     * convention is used by update_attitude(). Roll positive right-down,
     * pitch positive nose-up and yaw positive CW when viewed from above.
     *
     * theta_dot = q*cos(phi) - r*sin(phi)
     * phi_dot   = p + (q*sin(phi)+r*cos(phi))*tan(theta)
     * No separate yaw angle hold: yaw still tracks stick-rate input.
     */
    const float rad = 0.01745329251994329577f;
    const float phi = in->roll_deg * rad;
    const float theta = fmaxf(-1.3962634f, fminf(1.3962634f, in->pitch_deg * rad));
    const float cos_phi = cosf(phi);
    const float cos_phi_safe = (fabsf(cos_phi) < 0.174f) ?
        (cos_phi < 0.0f ? -0.174f : 0.174f) : cos_phi;
    const float phi_dot = limit(active_gains.angle_kp * (in->roll_set_deg-in->roll_deg),active_gains.max_rate_dps);
    const float theta_dot = limit(active_gains.angle_kp * (in->pitch_set_deg-in->pitch_deg),active_gains.max_rate_dps);
    const float r = in->gyro_yaw_dps;
    const float q_target = (theta_dot + r * sinf(phi)) / cos_phi_safe;
    const float p_target = phi_dot - (q_target * sinf(phi) + r * cos_phi) * tanf(theta);
    const float target[] = {
        limit(p_target, active_gains.max_rate_dps),
        limit(q_target, active_gains.max_rate_dps),
        limit(in->yaw_rate_set_dps, active_gains.max_rate_dps)
    };
    const float *kp = active_gains.rate_kp;
    float u[3];
    for (unsigned i=0;i<3;++i) {
        const float error = target[i] - gyro[i];
        const float d_alpha = 1.0f - expf(-PID_TAU_TWO_PI * PID_DTERM_LPF_HZ * in->dt_s);
        const float derivative = s->has_prev ? (gyro[i] - s->gyro_prev[i]) / in->dt_s : 0.0f;
        s->gyro_deriv_filt[i] += d_alpha * (derivative - s->gyro_deriv_filt[i]);
        const float p_term = kp[i] * error;
        const float d_term = -active_gains.rate_kd[i] * s->gyro_deriv_filt[i];
        if (in->reset_integrator) {
            s->integrator[i] = 0.0f;
        } else if (in->allow_integrator && active_gains.rate_ki[i] > 0.0f) {
            const float candidate_i = limit(s->integrator[i] + error * in->dt_s,
                                            active_gains.integral_limit);
            const float candidate_u = p_term + active_gains.rate_ki[i] * candidate_i + d_term;
            /* Conditional integration: a saturated axis may ONLY unwind.
             * The old fabs(unconstrained) check froze recovery on the rail. */
            if ((candidate_u < active_gains.correction_limit || error < 0.0f) &&
                (candidate_u > -active_gains.correction_limit || error > 0.0f)) {
                s->integrator[i] = candidate_i;
            }
        } else {
            /* Brief fast yaw/tilt transients freeze rather than ERASE the
             * bounded I bias.  Motor-off resets it above and ARM/disarm
             * still runs FlightPID_Reset.  Do not integrate against tethers. */
        }
        const float i_term = active_gains.rate_ki[i] * s->integrator[i];
        u[i] = limit(p_term + i_term + d_term, active_gains.correction_limit);
        s->rate_set_dps[i] = target[i];
        s->rate_error_dps[i] = error;
        s->pid_p[i] = p_term;
        s->pid_i[i] = i_term;
        s->pid_d[i] = d_term;
        s->gyro_prev[i] = gyro[i];
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

