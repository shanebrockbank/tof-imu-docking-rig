#ifndef HAL_HAL_HOST_H
#define HAL_HAL_HOST_H

#include <stdint.h>
#include "hal/hal.h"
#include "sim/sim_cart.h"
#include "sim/sim_range_sensor.h"
#include "sim/sim_imu.h"

typedef struct {
    sim_cart_t cart;
    sim_range_sensor_t range_sensor;
    sim_imu_t imu;
    double true_theta_rad;
    double true_omega_y_rps; /* current tick's ground-truth pitch rate, for IMU sim */
    double true_accel_mps2;  /* current tick's ground-truth longitudinal accel, for IMU sim */
    double clock_now_s;
    double last_servo_angle_deg;
} hal_host_world_t;

void hal_host_world_init(hal_host_world_t *w, double initial_range_m, double track_length_m, uint32_t seed);

/* Test/main-loop driver: advances the world by one tick BEFORE the caller
   reads through hal_t for that tick. */
void hal_host_world_tick(hal_host_world_t *w, double true_accel_mps2, double true_omega_y_rps, double dt);

hal_t hal_host_create(hal_host_world_t *w);

#endif /* HAL_HAL_HOST_H */
