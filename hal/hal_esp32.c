/*
 * Real ESP32 HAL backend. This is the ONLY file in this repo permitted to
 * #include ESP-IDF headers (CLAUDE.md constraint 1) — hal_esp32.h re-exports
 * none of them. See
 * docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md.
 *
 * Real, hardware-verified-good: ToF (VL53L1X) timing/distance-mode
 * constants, the ICM20948 register init/read sequence, the esp_timer-based
 * clock and busy-wait pacer. NOT yet verified against real hardware
 * (explicit, loudly-commented placeholders at their definition sites): the
 * IMU axis/sign mapping (hardware_bringup Test 2 was never run) and the
 * servo GPIO/pulse-width/frequency constants (no servo picked/wired yet).
 */

#include "hal/hal_esp32.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "hal/vl53l1x_status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <math.h>
#include <stdio.h>

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
#define SERVO_PWM_RESOLUTION_BITS 14
#define SERVO_MIN_PULSE_US   500.0
#define SERVO_MAX_PULSE_US   2500.0
#define SERVO_MIN_DEG        0.0
#define SERVO_MAX_DEG        180.0

static bool s_servo_ready = false;

static int64_t s_pace_iter_start_us = 0;

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   21
#define I2C_SCL_GPIO   22
#define I2C_FREQ_HZ    400000
#define I2C_TIMEOUT_MS 100

#define TOF_I2C_ADDR         0x29
#define TOF_DEV              ((uint16_t)(TOF_I2C_ADDR << 1))
#define TOF_DISTANCE_MODE    1    /* short, <=~1.3m — matches the 1.0m rail, confirmed by bring-up Test 5 */
#define TOF_TIMING_BUDGET_MS 33
#define TOF_INTER_MEAS_MS    50   /* ~20Hz, confirmed by bring-up Test 4 */

#define IMU_I2C_ADDR     0x69  /* ICM20948, matches current bring-up wiring (AD0 high) */
#define ACCEL_LSB_PER_G  16384.0
#define GYRO_LSB_PER_DPS 131.0
#define G_MPS2           9.81
#define DEG2RAD          (M_PI / 180.0)

static i2c_master_bus_handle_t s_i2c_bus;
static bool s_tof_ready = false;

static i2c_master_dev_handle_t s_imu_dev;
static bool s_imu_ready = false;

/* Forward declaration: esp32_range_read (below) calls esp32_clock_now,
   whose definition sits later in this file (unchanged from prior tasks). */
static timestamp_t esp32_clock_now(void *ctx);

static bool i2c_bus_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, &s_i2c_bus) == ESP_OK;
}

static void tof_init(void) {
    vl53l1_platform_bind(s_i2c_bus, TOF_I2C_ADDR);
    if (VL53L1X_SensorInit(TOF_DEV) != 0) return;
    VL53L1X_SetDistanceMode(TOF_DEV, TOF_DISTANCE_MODE);
    VL53L1X_SetTimingBudgetInMs(TOF_DEV, TOF_TIMING_BUDGET_MS);
    VL53L1X_SetInterMeasurementInMs(TOF_DEV, TOF_INTER_MEAS_MS);
    VL53L1X_StartRanging(TOF_DEV);
    s_tof_ready = true;
}

static hal_status_t esp32_range_read(void *ctx, range_sample_t *out) {
    (void)ctx;
    if (!s_tof_ready) return HAL_FAULT;

    uint8_t ready = 0;
    VL53L1X_CheckForDataReady(TOF_DEV, &ready);
    if (!ready) return vl53l1x_translate_status(false, 0);

    uint16_t distance_mm;
    uint8_t range_status;
    VL53L1X_GetDistance(TOF_DEV, &distance_mm);
    VL53L1X_GetRangeStatus(TOF_DEV, &range_status);
    VL53L1X_ClearInterrupt(TOF_DEV);

    hal_status_t status = vl53l1x_translate_status(true, range_status);
    if (status == HAL_OK) {
        out->range_m = distance_mm / 1000.0;
        out->ts = esp32_clock_now(NULL);
    }
    return status;
}

static uint8_t reg_read8(i2c_master_dev_handle_t dev, uint8_t reg) {
    uint8_t val = 0xFF;
    i2c_master_transmit_receive(dev, &reg, 1, &val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return val;
}

static void reg_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t v) {
    uint8_t buf[2] = { reg, v };
    i2c_master_transmit(dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t reg_read_bytes(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static void icm_select_bank(i2c_master_dev_handle_t dev, uint8_t bank) {
    reg_write8(dev, 0x7F, (bank & 0x03) << 4);
}

static void imu_init(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_imu_dev) != ESP_OK) return;

    icm_select_bank(s_imu_dev, 0);
    reg_write8(s_imu_dev, 0x06, 0x80); /* PWR_MGMT_1: DEVICE_RESET */
    vTaskDelay(pdMS_TO_TICKS(100));
    icm_select_bank(s_imu_dev, 0);
    reg_write8(s_imu_dev, 0x06, 0x01); /* PWR_MGMT_1: auto clock select, sleep=0 */
    reg_write8(s_imu_dev, 0x07, 0x00); /* PWR_MGMT_2: enable accel + gyro */
    vTaskDelay(pdMS_TO_TICKS(50));
    icm_select_bank(s_imu_dev, 2);
    reg_write8(s_imu_dev, 0x01, 0x01); /* GYRO_CONFIG_1: +-250dps, DLPF enabled */
    reg_write8(s_imu_dev, 0x14, 0x01); /* ACCEL_CONFIG: +-2g, DLPF enabled */
    icm_select_bank(s_imu_dev, 0);

    if (reg_read8(s_imu_dev, 0x00) != 0xEA) return; /* WHO_AM_I mismatch */
    s_imu_ready = true;
}

static hal_status_t translate_esp_err_to_hal_status(esp_err_t err) {
    return (err == ESP_OK) ? HAL_OK : HAL_FAULT;
}

static hal_status_t esp32_imu_read(void *ctx, imu_sample_t *out) {
    (void)ctx;
    if (!s_imu_ready) return HAL_FAULT;

    uint8_t buf[14];
    esp_err_t err = reg_read_bytes(s_imu_dev, 0x2D, buf, 14);
    if (translate_esp_err_to_hal_status(err) != HAL_OK) return HAL_FAULT;

    int16_t rax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ray = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raz = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t rgx = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t rgy = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t rgz = (int16_t)((buf[10] << 8) | buf[11]);

    /* PLACEHOLDER axis/sign mapping — hardware_bringup Test 2 (axis/sign
       mapping) has never been run. This identity mapping (chip X/Z -> HAL
       longitudinal/vertical, chip gyro-Y -> HAL pitch) is UNVERIFIED.
       Correct these indices/signs against real Test 2 results before
       trusting this backend's estimator output — see the approved design
       spec §3.2/§8. */
    out->accel_mps2[0] = (rax / ACCEL_LSB_PER_G) * G_MPS2; /* longitudinal, PLACEHOLDER */
    out->accel_mps2[1] = (ray / ACCEL_LSB_PER_G) * G_MPS2; /* lateral, unused */
    out->accel_mps2[2] = (raz / ACCEL_LSB_PER_G) * G_MPS2; /* vertical, PLACEHOLDER */
    out->gyro_rps[0] = (rgx / GYRO_LSB_PER_DPS) * DEG2RAD; /* unused */
    out->gyro_rps[1] = (rgy / GYRO_LSB_PER_DPS) * DEG2RAD; /* pitch, PLACEHOLDER */
    out->gyro_rps[2] = (rgz / GYRO_LSB_PER_DPS) * DEG2RAD; /* unused */
    out->ts = esp32_clock_now(NULL);
    return HAL_OK;
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
    uint32_t max_duty = (1u << SERVO_PWM_RESOLUTION_BITS) - 1u;
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
    servo_init();
    bool i2c_ready = i2c_bus_init();
    if (i2c_ready) {
        tof_init();
        imu_init();
    }

    printf("hal_esp32: servo=%s i2c_bus=%s tof=%s imu=%s\n",
           s_servo_ready ? "ready" : "FAILED",
           i2c_ready ? "ready" : "FAILED",
           s_tof_ready ? "ready" : "FAILED",
           s_imu_ready ? "ready" : "FAILED");

    s_pace_iter_start_us = esp_timer_get_time();
    hal_t h;
    h.ctx = NULL;
    h.range_read = esp32_range_read;
    h.imu_read = esp32_imu_read;
    h.actuator_set_angle_deg = esp32_actuator_set_angle_deg;
    h.clock_now = esp32_clock_now;
    h.pace_tick = esp32_pace_tick;
    return h;
}
