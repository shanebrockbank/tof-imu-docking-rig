#ifndef SIM_SIM_IMU_H
#define SIM_SIM_IMU_H

#include <stdint.h>
#include "common/types.h"
#include "common/measurement.h"
#include "sim/sim_noise.h"

#define SIM_GRAVITY_MPS2 9.81

typedef struct {
    double period_s;           /* nominal IMU sample period, e.g. 0.01 (100Hz) */
    double accel_noise_stddev;
    double gyro_noise_stddev;
    double accel_bias_x_mps2;  /* test hook, default 0 */
    double last_emit_t_s;
    sim_noise_t noise;
} sim_imu_t;

void sim_imu_init(sim_imu_t *s, double period_s, double accel_noise_stddev, double gyro_noise_stddev, uint32_t seed);
void sim_imu_set_accel_bias(sim_imu_t *s, double bias_mps2);

hal_status_t sim_imu_sample(sim_imu_t *s, double true_accel_long_mps2, double true_theta_rad, double true_omega_y_rps, timestamp_t now, imu_sample_t *out);

#endif /* SIM_SIM_IMU_H */
