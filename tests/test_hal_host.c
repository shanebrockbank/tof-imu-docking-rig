#include "tests/test_framework.h"
#include "hal/hal_host.h"

TEST(test_hal_t_reads_route_through_world_state) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 1);
    hal_t h = hal_host_create(&w);

    hal_host_world_tick(&w, 0.0, 0.0, 0.01);
    range_sample_t r;
    CHECK(h.range_read(h.ctx, &r) == HAL_OK); /* first read always due */
    CHECK_NEAR(r.range_m, 1.0, 0.02);

    imu_sample_t s;
    CHECK(h.imu_read(h.ctx, &s) == HAL_OK);
}

TEST(test_actuator_command_is_captured) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 1);
    hal_t h = hal_host_create(&w);
    CHECK(h.actuator_set_angle_deg(h.ctx, 120.0) == HAL_OK);
    CHECK_NEAR(w.last_servo_angle_deg, 120.0, 1e-9);
}

TEST(test_clock_now_reflects_ticks) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 1);
    hal_t h = hal_host_create(&w);
    hal_host_world_tick(&w, 0.0, 0.0, 0.01);
    hal_host_world_tick(&w, 0.0, 0.0, 0.01);
    CHECK_NEAR(h.clock_now(h.ctx).t_s, 0.02, 1e-9);
}

TEST(test_tick_advances_true_cart_state) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 1);
    hal_host_world_tick(&w, 1.0, 0.0, 0.1);
    CHECK(w.cart.true_range_m < 1.0);
    CHECK(w.cart.true_velocity_mps > 0.0);
}

TEST(test_pace_tick_is_noop) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 1);
    hal_t h = hal_host_create(&w);
    hal_host_world_tick(&w, 0.0, 0.0, 0.01);
    timestamp_t before = h.clock_now(h.ctx);
    h.pace_tick(h.ctx);
    timestamp_t after = h.clock_now(h.ctx);
    CHECK_NEAR(before.t_s, after.t_s, 1e-9);
}

int main(void) {
    RUN_TEST(test_hal_t_reads_route_through_world_state);
    RUN_TEST(test_actuator_command_is_captured);
    RUN_TEST(test_clock_now_reflects_ticks);
    RUN_TEST(test_tick_advances_true_cart_state);
    RUN_TEST(test_pace_tick_is_noop);
    TEST_SUMMARY();
    return 0;
}
