#include "hal/hal_host.h"

#define HAL_HOST_TOF_PERIOD_S 0.05
#define HAL_HOST_TOF_NOISE_STDDEV_M 0.003
#define HAL_HOST_IMU_PERIOD_S 0.01
#define HAL_HOST_IMU_ACCEL_NOISE_STDDEV 0.05
#define HAL_HOST_IMU_GYRO_NOISE_STDDEV 0.01

void hal_host_world_init(hal_host_world_t *w, double initial_range_m, double track_length_m, uint32_t seed) {
    sim_cart_init(&w->cart, initial_range_m, track_length_m);
    sim_range_sensor_init(&w->range_sensor, HAL_HOST_TOF_PERIOD_S, HAL_HOST_TOF_NOISE_STDDEV_M, seed);
    sim_imu_init(&w->imu, HAL_HOST_IMU_PERIOD_S, HAL_HOST_IMU_ACCEL_NOISE_STDDEV, HAL_HOST_IMU_GYRO_NOISE_STDDEV, seed + 1);
    w->true_theta_rad = 0.0;
    w->true_omega_y_rps = 0.0;
    w->true_accel_mps2 = 0.0;
    w->clock_now_s = 0.0;
    w->last_servo_angle_deg = 90.0;
}

void hal_host_world_tick(hal_host_world_t *w, double true_accel_mps2, double true_omega_y_rps, double dt) {
    w->clock_now_s += dt;
    sim_cart_step(&w->cart, true_accel_mps2, dt);
    w->true_theta_rad += true_omega_y_rps * dt;
    w->true_omega_y_rps = true_omega_y_rps;
    w->true_accel_mps2 = true_accel_mps2;
}

static hal_status_t host_range_read(void *ctx, range_sample_t *out) {
    hal_host_world_t *w = (hal_host_world_t *)ctx;
    timestamp_t now = { w->clock_now_s };
    return sim_range_sensor_sample(&w->range_sensor, w->cart.true_range_m, now, out);
}

static hal_status_t host_imu_read(void *ctx, imu_sample_t *out) {
    hal_host_world_t *w = (hal_host_world_t *)ctx;
    timestamp_t now = { w->clock_now_s };
    return sim_imu_sample(&w->imu, w->true_accel_mps2, w->true_theta_rad, w->true_omega_y_rps, now, out);
}

static hal_status_t host_actuator_set_angle_deg(void *ctx, double angle_deg) {
    hal_host_world_t *w = (hal_host_world_t *)ctx;
    w->last_servo_angle_deg = angle_deg;
    return HAL_OK;
}

static timestamp_t host_clock_now(void *ctx) {
    hal_host_world_t *w = (hal_host_world_t *)ctx;
    timestamp_t t = { w->clock_now_s };
    return t;
}

static void host_pace_tick(void *ctx) {
    (void)ctx; /* host/sim has no wall clock to wait on; time advances via hal_host_world_tick() */
}

hal_t hal_host_create(hal_host_world_t *w) {
    hal_t h;
    h.ctx = w;
    h.range_read = host_range_read;
    h.imu_read = host_imu_read;
    h.actuator_set_angle_deg = host_actuator_set_angle_deg;
    h.clock_now = host_clock_now;
    h.pace_tick = host_pace_tick;
    return h;
}
