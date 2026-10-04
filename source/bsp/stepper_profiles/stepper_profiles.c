#include <math.h>
#include <stdint.h>

#include "esp_attr.h"

#include "stepper_profiles.h"

#ifdef __cplusplus
extern "C" {
#endif

trapezoidal_profile_t plan_trapezoidal_motion(
    uint32_t total_steps, 
    float start_speed_rev_s,
    float desired_speed_rev_s,
    float desired_accel_rev_s2, 
    uint32_t steps_per_rev
) 
{
    trapezoidal_profile_t motion = {};
    if (total_steps == 0 || steps_per_rev == 0) return motion;

    if (desired_speed_rev_s < start_speed_rev_s || desired_accel_rev_s2 <= 0.0f) {
        return motion; // Invalid parameters, return empty profile
    }

    // 1. Convert physical inputs (rev/s, rev/s²) to step-domain units (steps/s, steps/s²)
    float steps_scale = (float)(steps_per_rev);

    float v0        = start_speed_rev_s * steps_scale;       // steps/s
    float v_target  = desired_speed_rev_s * steps_scale;     // steps/s
    float a_max     = desired_accel_rev_s2 * steps_scale;   // steps/s²

    // 2. Nominal step calculation for pure acceleration phase (Torricelli's Law in step-domain)
    float delta_v2 = (v_target * v_target) - (v0 * v0);
    float n_accel_float = delta_v2 / (2.0f * a_max);
    
    uint32_t n_accel_calc = (uint32_t)(round(n_accel_float));

    float v_peak = v_target;

    // 3. Triangular vs. Trapezoidal Profile Check
    if (n_accel_calc > total_steps / 2) {
        float available_accel_steps = (float)(total_steps / 2);
        float v_peak_sq = (v0 * v0) + 2.0f * a_max * available_accel_steps;
        v_peak = sqrt(fmax(0.0f, v_peak_sq));

        motion.n_accel  = total_steps / 2;
        motion.n_cruise = total_steps % 2; 
    } else {
        motion.n_accel  = n_accel_calc;
        motion.n_cruise = total_steps - (2 * motion.n_accel);
    }

    // 4. Fill metadata converted back to physical units (rev/s, rev/s²)
    motion.required_steps = total_steps;
    motion.max_speed_rev_s = v_peak / steps_scale;
    motion.max_acceleration_rev_s2 = a_max / steps_scale;

    return motion;
}

trapezoidal_david_austin_profile_t plan_trapezoidal_david_austin_motion(
    uint32_t total_steps,
    float start_speed_rev_s,
    float desired_speed_rev_s,
    float desired_accel_rev_s2, 
    uint32_t steps_per_rev, 
    uint32_t resolution_freq_hz
)
{
    trapezoidal_david_austin_profile_t profile = {};

    if (total_steps == 0 || steps_per_rev == 0 || resolution_freq_hz == 0 ||
        desired_speed_rev_s <= 0.0f || desired_accel_rev_s2 <= 0.0f) return profile;

    trapezoidal_profile_t plan = plan_trapezoidal_motion(
        total_steps, start_speed_rev_s, desired_speed_rev_s,
        desired_accel_rev_s2, steps_per_rev);
    if (plan.required_steps == 0) return profile;

    const float scale = (float)(steps_per_rev);
    const float f     = (float)(resolution_freq_hz);
    const float v0    = start_speed_rev_s * scale;
    const float a     = desired_accel_rev_s2 * scale;

    profile.total_steps = plan.required_steps;
    profile.n_accel     = plan.n_accel;
    profile.n_cruise    = plan.n_cruise;
    profile.start_n     = (uint32_t)(round(v0 * v0 / (2.0f * a)));
    profile.current_step = 0;
    if (profile.start_n == 0) {
        const float t1 = (-v0 + sqrt(v0 * v0 + 2.0f * a)) / a;
        const float t2 = (-v0 + sqrt(v0 * v0 + 4.0f * a)) / a;
        profile.c  = (uint32_t)(round(f * t1));
        profile.c1 = (uint32_t)(round(f * (t2 - t1)));
    } else {
        profile.c  = (uint32_t)(round(f / v0));
        profile.c1 = 0;
    }
    return profile;
}

s_curve_profile_t plan_s_curve_motion(
    uint32_t total_steps, 
    float start_speed_rev_s, 
    float desired_speed_rev_s, 
    float desired_accel_rev_s2, 
    float desired_jerk_rev_s3,
    uint32_t steps_per_rev
)
{
    s_curve_profile_t motion = {};
    if (total_steps == 0 || steps_per_rev == 0) return motion;

    if (desired_speed_rev_s <= start_speed_rev_s) {
        desired_speed_rev_s = start_speed_rev_s + 0.01f;
    }

    // 1. Convert physical units (rev/s, rev/s², rev/s³) into step units (steps/s, steps/s², steps/s³)
    float steps_scale = (float)(steps_per_rev);
    
    float v0        = start_speed_rev_s * steps_scale;       // steps/s
    float v_target  = desired_speed_rev_s * steps_scale;     // steps/s
    float a_max     = desired_accel_rev_s2 * steps_scale;   // steps/s²
    float j         = desired_jerk_rev_s3 * steps_scale;    // steps/s³

    // 2. Check if velocity delta allows reaching max acceleration in step-domain
    float delta_v_target = v_target - v0;
    float delta_v_min_a = (a_max * a_max) / j;

    float t_j, t_a, a_peak;

    if (delta_v_target < delta_v_min_a) {
        // Short profile: Does not reach a_max (Phase 2 is eliminated)
        t_a = 0.0f;
        t_j = sqrt(delta_v_target / j);
        a_peak = j * t_j;
    } else {
        // Full acceleration profile: Reaches a_max
        t_j = a_max / j;
        t_a = (delta_v_target - delta_v_min_a) / a_max;
        a_peak = a_max;
    }

    // 3. Integrals of velocity in step-domain -> results directly in STEP COUNTS
    float n1_f = v0 * t_j + (j * t_j * t_j * t_j) / 6.0f;
    float v1_f = v0 + 0.5f * j * t_j * t_j;

    float n2_f = v1_f * t_a + 0.5f * a_peak * t_a * t_a;
    float v2_f = v1_f + a_peak * t_a;

    float n3_f = v2_f * t_j + 0.5f * a_peak * t_j * t_j - (j * t_j * t_j * t_j) / 6.0f;

    uint32_t n_accel_nominal = (uint32_t)(round(n1_f + n2_f + n3_f));

    float v_peak = v_target;

    // 4. Check distance limit (Triangular / No-Cruise profile in step-domain)
    if (2 * n_accel_nominal > total_steps) {
        if (delta_v_target < delta_v_min_a) {
            float term = (j * pow(total_steps - 2.0f * v0 * t_j, 2.0f)) / 4.0f;
            v_peak = v0 + cbrt(term);
            t_j = sqrt((v_peak - v0) / j);
            t_a = 0.0f;
            a_peak = j * t_j;
        } else {
            float T_accel_approx = 2.0f * t_j;
            v_peak = v0 + a_max * sqrt((total_steps - v0 * T_accel_approx) / a_max + t_j * t_j) - a_max * t_j;
            t_a = (v_peak - v0 - delta_v_min_a) / a_max;
            if (t_a < 0.0f) t_a = 0.0f;
            a_peak = a_max;
        }

        // Recalculate discrete step areas based on adjusted peak limits
        n1_f = v0 * t_j + (j * t_j * t_j * t_j) / 6.0f;
        v1_f = v0 + 0.5f * j * t_j * t_j;
        n2_f = v1_f * t_a + 0.5f * a_peak * t_a * t_a;
        v2_f = v1_f + a_peak * t_a;
        n3_f = v2_f * t_j + 0.5f * a_peak * t_j * t_j - (j * t_j * t_j * t_j) / 6.0f;
    }

    // 5. Assign rounded integer steps to output phases
    motion.n_1 = (uint32_t)(round(n1_f));
    motion.n_2 = (uint32_t)(round(n2_f));
    motion.n_3 = (uint32_t)(round(n3_f));

    uint32_t n_accel_decel_total = 2 * (motion.n_1 + motion.n_2 + motion.n_3);

    // 6. Cruise phase absorbs the remaining total steps
    if (total_steps >= n_accel_decel_total) {
        motion.n_4 = total_steps - n_accel_decel_total;
    } else {
        motion.n_4 = 0;
        int32_t diff = n_accel_decel_total - total_steps;
        if (diff > 0 && motion.n_2 >= (uint32_t)(diff / 2)) {
            motion.n_2 -= diff / 2;
        }
    }

    // 7. Store final parameters back in physical units (rev/s, rev/s², rev/s³)
    motion.required_steps = total_steps;
    motion.max_speed_rev_s = v_peak / steps_scale;
    motion.max_acceleration_rev_s2 = a_peak / steps_scale;
    motion.max_jerk_rev_s3 = j / steps_scale;

    return motion;
}

bool trapezoidal_profile_next_step(trapezoidal_profile_t *profile, uint32_t *delta_us)
{
    // 1. Check if motion is complete
    if (profile->current_step >= profile->required_steps) {
        *delta_us = 0;
        return false;
    }

    uint32_t n = profile->current_step;
    uint32_t accel_end = profile->n_accel;
    uint32_t cruise_end = profile->n_accel + profile->n_cruise;
    float next_speed;

    // 2. Calculate instantaneous speed based on active motion phase
    if (n < accel_end) {
        // Acceleration phase: v^2 = v0^2 + 2 * a * n
        float v_sq = (profile->start_speed_rev_s * profile->start_speed_rev_s) +
                     (2.0f * profile->max_acceleration_rev_s2 * (float)(n + 1));
        next_speed = sqrtf(v_sq);
    } 
    else if (n < cruise_end) {
        // Cruise phase: constant speed
        next_speed = profile->max_speed_rev_s;
    } 
    else {
        // Deceleration phase: mirrored step count from end
        uint32_t decel_step = profile->required_steps - 1 - n;
        float v_sq = (profile->start_speed_rev_s * profile->start_speed_rev_s) +
                     (2.0f * profile->max_acceleration_rev_s2 * (float)decel_step);
        next_speed = sqrtf(v_sq);
    }

    // Zero-division protection
    if (next_speed <= 0.0f) {
        next_speed = (profile->start_speed_rev_s > 0.0f) ? profile->start_speed_rev_s : 1.0f;
    }

    // 3. Compute time interval for current step (dt = 1 / speed)
    *delta_us = (uint32_t)(1'000'000.0f / next_speed);

    // 4. Advance step count
    profile->current_step++;

    return true;
}

bool IRAM_ATTR trapezoidal_david_austin_profile_next_step(trapezoidal_david_austin_profile_t * p)
{
    if (p->current_step >= p->total_steps) return false;

    p->current_step++;
    const uint32_t idx = p->current_step;

    const uint32_t end_accel  = p->n_accel;
    const uint32_t end_cruise = p->n_accel + p->n_cruise;

    if (idx <= end_accel) {
        if (p->start_n == 0 && idx == 1) {
            p->c    = p->c1;
            p->rest = 0;
        } else {
            const uint32_t d   = 4u * (p->start_n + idx) + 1u;
            const uint32_t num = 2u * p->c + p->rest;
            p->c   -= num / d;
            p->rest = num % d;
        }
    } else if (idx <= end_cruise) {
    } else {
        const uint32_t j = idx - end_cruise;
        const uint32_t n = p->start_n + p->n_accel - j + 1u;
        if (j == 1) p->rest = 0;
        const uint32_t d   = 4u * n - 1u;
        const uint32_t num = 2u * p->c + p->rest;
        p->c   += num / d;
        p->rest = num % d;
    }

    return p->current_step <= p->total_steps;
}

#ifdef __cplusplus
}
#endif