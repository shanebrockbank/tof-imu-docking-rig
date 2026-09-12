/*
 * Real ESP32 HAL backend. This is the ONLY file in this repo permitted to
 * #include ESP-IDF headers (CLAUDE.md constraint 1) — hal_esp32.h re-exports
 * none of them. See
 * docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md.
 *
 * This is a scaffolding stub: every backend function is a placeholder that
 * compiles and links but does not yet talk to real hardware. Each is filled
 * in by a later task in that spec's implementation plan.
 */

#include "hal/hal_esp32.h"

#include <stddef.h>

static hal_status_t esp32_range_read(void *ctx, range_sample_t *out) {
    (void)ctx; (void)out;
    return HAL_FAULT;
}

static hal_status_t esp32_imu_read(void *ctx, imu_sample_t *out) {
    (void)ctx; (void)out;
    return HAL_FAULT;
}

static hal_status_t esp32_actuator_set_angle_deg(void *ctx, double angle_deg) {
    (void)ctx; (void)angle_deg;
    return HAL_FAULT;
}

static timestamp_t esp32_clock_now(void *ctx) {
    (void)ctx;
    timestamp_t t = { 0.0 };
    return t;
}

static void esp32_pace_tick(void *ctx) {
    (void)ctx;
}

hal_t hal_esp32_create(void) {
    hal_t h;
    h.ctx = NULL;
    h.range_read = esp32_range_read;
    h.imu_read = esp32_imu_read;
    h.actuator_set_angle_deg = esp32_actuator_set_angle_deg;
    h.clock_now = esp32_clock_now;
    h.pace_tick = esp32_pace_tick;
    return h;
}
