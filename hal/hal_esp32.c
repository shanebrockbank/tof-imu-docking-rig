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

#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/ledc.h"

#include <stddef.h>

#define MAIN_LOOP_PERIOD_US 10000 /* 100 Hz, docs/design.md §5.6 */

/* PLACEHOLDER servo constants — no servo hardware picked/wired yet (per
   the approved design spec §1.1/§3.3/§8). GPIO18 is a concrete default,
   free on the current wiring (distinct from I2C's GPIO21/22 and UART0's
   default 1/3) — change it at real wiring time if a different pin is more
   convenient. 50Hz/500-2500us are standard hobby-servo values; verify
   against the real servo once one exists. */
#define SERVO_GPIO           18
#define SERVO_PWM_FREQ_HZ    50
#define SERVO_PWM_TIMER      LEDC_TIMER_0
#define SERVO_PWM_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_PWM_CHANNEL    LEDC_CHANNEL_0
#define SERVO_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_MIN_PULSE_US   500.0
#define SERVO_MAX_PULSE_US   2500.0
#define SERVO_MIN_DEG        0.0
#define SERVO_MAX_DEG        180.0

static bool s_servo_ready = false;

static int64_t s_pace_iter_start_us = 0;

static hal_status_t esp32_range_read(void *ctx, range_sample_t *out) {
    (void)ctx; (void)out;
    return HAL_FAULT;
}

static hal_status_t esp32_imu_read(void *ctx, imu_sample_t *out) {
    (void)ctx; (void)out;
    return HAL_FAULT;
}

static void servo_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = SERVO_PWM_MODE,
        .timer_num = SERVO_PWM_TIMER,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) return;

    ledc_channel_config_t channel_cfg = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_PWM_MODE,
        .channel = SERVO_PWM_CHANNEL,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    if (ledc_channel_config(&channel_cfg) != ESP_OK) return;

    s_servo_ready = true;
}

static hal_status_t esp32_actuator_set_angle_deg(void *ctx, double angle_deg) {
    (void)ctx;
    if (!s_servo_ready) return HAL_FAULT;
    if (angle_deg < SERVO_MIN_DEG || angle_deg > SERVO_MAX_DEG) return HAL_FAULT;

    double frac = (angle_deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    double pulse_us = SERVO_MIN_PULSE_US + frac * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    double period_us = 1e6 / SERVO_PWM_FREQ_HZ;
    uint32_t max_duty = (1u << SERVO_PWM_RESOLUTION) - 1u;
    uint32_t duty = (uint32_t)(pulse_us / period_us * max_duty);

    if (ledc_set_duty(SERVO_PWM_MODE, SERVO_PWM_CHANNEL, duty) != ESP_OK) return HAL_FAULT;
    if (ledc_update_duty(SERVO_PWM_MODE, SERVO_PWM_CHANNEL) != ESP_OK) return HAL_FAULT;
    return HAL_OK;
}

static timestamp_t esp32_clock_now(void *ctx) {
    (void)ctx;
    timestamp_t t;
    t.t_s = (double)esp_timer_get_time() / 1e6;
    return t;
}

static void esp32_pace_tick(void *ctx) {
    (void)ctx;
    /* Busy-wait remainder-of-period pacing — the exact pattern proven
       reliable on this hardware/ESP-IDF combo in hardware_bringup Tests
       3/6. A vTaskDelay-based tick is deliberately NOT used here (known to
       trigger I2C software timeouts under sustained back-to-back
       transactions on this setup). */
    int64_t target = s_pace_iter_start_us + MAIN_LOOP_PERIOD_US;
    int64_t now = esp_timer_get_time();
    if (now < target) esp_rom_delay_us((uint32_t)(target - now));
    s_pace_iter_start_us = esp_timer_get_time();
}

hal_t hal_esp32_create(void) {
    s_pace_iter_start_us = esp_timer_get_time();
    servo_init();
    hal_t h;
    h.ctx = NULL;
    h.range_read = esp32_range_read;
    h.imu_read = esp32_imu_read;
    h.actuator_set_angle_deg = esp32_actuator_set_angle_deg;
    h.clock_now = esp32_clock_now;
    h.pace_tick = esp32_pace_tick;
    return h;
}
