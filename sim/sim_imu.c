#include "sim/sim_imu.h"
#include <stdbool.h>
#include <math.h>

/* Forward gravity model, chosen as the exact algebraic inverse of
   estimation/gravity_compensation.c's
       a_longitudinal = a_x*cos(theta) - a_z*sin(theta)
   Given world-frame specific force (true_accel_long, 0, g) (a stationary,
   level accelerometer reads +g upward due to normal force), rotating into
   the body frame by pitch theta about Y gives:
       a_x =  true_accel_long*cos(theta) + g*sin(theta)
       a_z = -true_accel_long*sin(theta) + g*cos(theta)
   Substituting back into the gravity-compensation formula recovers
   true_accel_long exactly (cos^2+sin^2=1), which is what
   tests/test_sim_imu.c and tests/test_gravity_compensation.c check
   end-to-end. */

void sim_imu_init(sim_imu_t *s, double period_s, double accel_noise_stddev, double gyro_noise_stddev, uint32_t seed) {
    s->period_s = period_s;
    s->accel_noise_stddev = accel_noise_stddev;
    s->gyro_noise_stddev = gyro_noise_stddev;
    s->accel_bias_x_mps2 = 0.0;
    s->last_emit_t_s = -1.0;
    sim_noise_seed(&s->noise, seed);
}

void sim_imu_set_accel_bias(sim_imu_t *s, double bias_mps2) {
    s->accel_bias_x_mps2 = bias_mps2;
}

hal_status_t sim_imu_sample(sim_imu_t *s, double true_accel_long_mps2, double true_theta_rad, double true_omega_y_rps, timestamp_t now, imu_sample_t *out) {
    bool due = (s->last_emit_t_s < 0.0) || (now.t_s - s->last_emit_t_s >= s->period_s);
    if (!due) {
        return HAL_NO_NEW_DATA;
    }
    s->last_emit_t_s = now.t_s;

    double a_x =  true_accel_long_mps2 * cos(true_theta_rad) + SIM_GRAVITY_MPS2 * sin(true_theta_rad);
    double a_z = -true_accel_long_mps2 * sin(true_theta_rad) + SIM_GRAVITY_MPS2 * cos(true_theta_rad);

    out->accel_mps2[0] = a_x + s->accel_bias_x_mps2 + sim_noise_gaussian(&s->noise, 0.0, s->accel_noise_stddev);
    out->accel_mps2[1] = 0.0;
    out->accel_mps2[2] = a_z + sim_noise_gaussian(&s->noise, 0.0, s->accel_noise_stddev);
    out->gyro_rps[0] = 0.0;
    out->gyro_rps[1] = true_omega_y_rps + sim_noise_gaussian(&s->noise, 0.0, s->gyro_noise_stddev);
    out->gyro_rps[2] = 0.0;
    out->ts = now;
    out->status = HAL_OK;
    return HAL_OK;
}
