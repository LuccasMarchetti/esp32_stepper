#ifndef STEPPER_PROFILES_H
#define STEPPER_PROFILES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t required_steps;            // Total steps required for the motion
    float start_speed_rev_s;            // Starting speed in Hz
    float max_speed_rev_s;              // Speed in Hz
    float max_acceleration_rev_s2;      // Acceleration in Hz/s
    uint32_t n_accel, n_cruise;         // Number of acceleration and cruise steps 
    uint32_t current_step;
} trapezoidal_profile_t;

typedef struct {
    uint32_t total_steps;
    uint32_t n_accel;
    uint32_t n_cruise;
    uint32_t current_step;
    uint32_t start_n;
    uint32_t c;
    uint32_t c1;
    uint32_t rest;
} trapezoidal_david_austin_profile_t;

typedef struct {
    uint32_t required_steps;            // Total steps required for the motion
    float start_speed_rev_s;            // Starting speed in Hz
    float max_speed_rev_s;              // Speed in Hz
    float max_acceleration_rev_s2;      // Acceleration in Hz/s
    float max_jerk_rev_s3;              // Jerk in Hz/s²
    uint32_t n_1, n_2, n_3, n_4;        // Number of steps per phase (jerk+, accel+, jerk-, cruise) mirrored for deceleration
    uint32_t current_step;
} s_curve_profile_t;

typedef struct {
    uint32_t total_steps;
    uint32_t n_1, n_2, n_3, n_4;
    uint32_t n_accel;
    uint32_t start_speed_rev_s_q16;
    uint32_t max_speed_rev_s_q16;
    uint32_t freq_p1_q16;
    uint32_t freq_p2_q16;
} s_curve_isr_profile_t;

trapezoidal_profile_t plan_trapezoidal_motion(
    uint32_t total_steps, 
    float start_speed_rev_s,
    float desired_speed_rev_s,
    float desired_accel_rev_s2, 
    uint32_t steps_per_rev
);

trapezoidal_david_austin_profile_t plan_trapezoidal_david_austin_motion(
    uint32_t total_steps,
    float start_speed_rev_s,
    float desired_speed_rev_s,
    float desired_accel_rev_s2, 
    uint32_t steps_per_rev, 
    uint32_t resolution_freq_hz
);

s_curve_profile_t plan_s_curve_motion(
    uint32_t total_steps, 
    float start_speed_rev_s, 
    float desired_speed_rev_s, 
    float desired_accel_rev_s2, 
    float desired_jerk_rev_s3,
    uint32_t steps_per_rev
);

bool trapezoidal_profile_next_step(trapezoidal_profile_t *profile, uint32_t *delta_us);
bool trapezoidal_david_austin_profile_next_step(trapezoidal_david_austin_profile_t * p);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_PROFILES_H