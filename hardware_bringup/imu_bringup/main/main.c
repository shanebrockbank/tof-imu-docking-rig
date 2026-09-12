/*
 * Standalone IMU + ToF bring-up firmware — hardware_bringup Tests 1-6.
 * See hardware_bringup/README.md. Not part of the V1/V2 pipeline. Plain
 * ESP-IDF I2C master driver, no Arduino/Wire — works unmodified with either
 * an MPU6050 or ICM20948 wired to the default I2C pins (auto-detected via
 * WHO_AM_I at boot — swap chips and reset, no rebuild needed), sharing the
 * bus with a VL53L1X ToF sensor at its default address via the vendored
 * Ultra Lite Driver in components/vl53l1x_uld/ (see that directory's
 * README.md for what's vendored vs. written for this project).
 *
 * UART0 console @ 115200. Menu-driven: '1'-'6' run a test, 's' stops the
 * live-stream test (2) or the interactive test (5).
 *
 * NOTE: this file intentionally includes ESP-IDF headers, which CLAUDE.md
 * constraint #1 otherwise reserves for hal/hal_esp32.c alone. That's a
 * deliberate, scoped exception for this standalone bring-up firmware (per
 * user direction) - it is not part of the V1/V2 pipeline and is deleted
 * once hardware bring-up is complete, so it never coexists with the
 * mechanical hal-boundary check the constraint describes.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "VL53L1X_api.h"
#include "vl53l1_platform.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   21
#define I2C_SCL_GPIO   22
#define I2C_FREQ_HZ    400000
#define I2C_TIMEOUT_MS 100

// Both chips are configured to +-2g / +-250dps below, so one pair of
// sensitivity constants covers both.
#define ACCEL_LSB_PER_G  16384.0
#define GYRO_LSB_PER_DPS 131.0
#define G_MPS2           9.81
#define DEG2RAD          (M_PI / 180.0)

// Main-loop poll rate per docs/design.md §5.6 -- used to pace Tests 4 and 6.
#define MAIN_LOOP_PERIOD_US 10000 // 100Hz

// VL53L1X ToF sensor config. Distance mode 1=short (<=~1.3m), 2=long
// (<=~4m) -- short chosen since the docking rig's rail is 1.0m; Test 5
// (noise across range) is exactly what should confirm or revisit this.
// timing_budget/inter_measurement follow ST's own example pairing for a
// ~20Hz rate, matching docs/design.md's ToF-nominal-rate assumption.
#define TOF_I2C_ADDR         0x29 // VL53L1X default 7-bit address
#define TOF_DISTANCE_MODE    1
#define TOF_TIMING_BUDGET_MS 33
#define TOF_INTER_MEAS_MS    50

typedef enum { CHIP_NONE, CHIP_MPU6050, CHIP_ICM20948 } chip_type_t;

typedef struct {
  chip_type_t chip;
  uint8_t addr;
  i2c_master_dev_handle_t dev;
} imu_device_t;

static i2c_master_bus_handle_t s_bus;

#define MAX_DEVICES 2
static imu_device_t s_devices[MAX_DEVICES];
static int s_device_count = 0;

static const char *chip_name(chip_type_t chip) {
  return chip == CHIP_MPU6050 ? "MPU6050" : "ICM20948";
}

// ---- Low-level register helpers -------------------------------------------

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

// ---- I2C bus / chip detect + init ------------------------------------------

static void i2c_bus_init(void) {
  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_PORT,
    .sda_io_num = I2C_SDA_GPIO,
    .scl_io_num = I2C_SCL_GPIO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_bus));
}

static void detect_devices(void) {
  const uint8_t candidates[2] = { 0x68, 0x69 };
  s_device_count = 0;
  for (int i = 0; i < 2 && s_device_count < MAX_DEVICES; i++) {
    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = candidates[i],
      .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &dev) != ESP_OK) continue;

    if (reg_read8(dev, 0x75) == 0x68) { // MPU6050 WHO_AM_I
      s_devices[s_device_count++] = (imu_device_t){ CHIP_MPU6050, candidates[i], dev };
      continue;
    }

    icm_select_bank(dev, 0);
    if (reg_read8(dev, 0x00) == 0xEA) { // ICM20948 WHO_AM_I
      s_devices[s_device_count++] = (imu_device_t){ CHIP_ICM20948, candidates[i], dev };
      continue;
    }

    i2c_master_bus_rm_device(dev);
  }
}

static void init_mpu6050(i2c_master_dev_handle_t dev) {
  reg_write8(dev, 0x6B, 0x00); // PWR_MGMT_1: wake from sleep
  vTaskDelay(pdMS_TO_TICKS(50));
  reg_write8(dev, 0x1A, 0x01); // CONFIG: DLPF ~188Hz gyro / 184Hz accel bandwidth
  reg_write8(dev, 0x1B, 0x00); // GYRO_CONFIG: FS_SEL=0 -> +-250 dps
  reg_write8(dev, 0x1C, 0x00); // ACCEL_CONFIG: AFS_SEL=0 -> +-2g
  reg_write8(dev, 0x19, 0x00); // SMPLRT_DIV=0 -> 1kHz internal sample rate
}

static void init_icm20948(i2c_master_dev_handle_t dev) {
  icm_select_bank(dev, 0);
  reg_write8(dev, 0x06, 0x80); // PWR_MGMT_1: DEVICE_RESET
  vTaskDelay(pdMS_TO_TICKS(100));
  icm_select_bank(dev, 0); // bank selection resets too; reselect to be safe
  reg_write8(dev, 0x06, 0x01); // PWR_MGMT_1: auto clock select, sleep=0
  reg_write8(dev, 0x07, 0x00); // PWR_MGMT_2: enable accel + gyro
  vTaskDelay(pdMS_TO_TICKS(50));
  icm_select_bank(dev, 2);
  reg_write8(dev, 0x01, 0x01); // GYRO_CONFIG_1: FS_SEL=+-250dps, DLPF enabled
  reg_write8(dev, 0x14, 0x01); // ACCEL_CONFIG: FS_SEL=+-2g, DLPF enabled
  icm_select_bank(dev, 0);
}

static void init_device(imu_device_t *d) {
  if (d->chip == CHIP_MPU6050) init_mpu6050(d->dev);
  else if (d->chip == CHIP_ICM20948) init_icm20948(d->dev);
}

// ---- ToF sensor (VL53L1X) init ----------------------------------------------

// "dev" here is the conventional VL53L1X ULD 8-bit-address value threaded
// through every VL53L1X_* call -- our platform shim (vl53l1_platform.c)
// binds directly to one device handle and ignores it, but every call still
// carries a real value for consistency with ST's own examples/docs.
static const uint16_t TOF_DEV = TOF_I2C_ADDR << 1;

static bool s_tof_present = false;

static void tof_init(void) {
  vl53l1_platform_bind(s_bus, TOF_I2C_ADDR);

  if (VL53L1X_SensorInit(TOF_DEV) != 0) {
    printf("ToF: SensorInit failed -- check wiring (SDA=%d, SCL=%d, addr=0x%02X).\n",
           I2C_SDA_GPIO, I2C_SCL_GPIO, TOF_I2C_ADDR);
    return;
  }
  VL53L1X_SetDistanceMode(TOF_DEV, TOF_DISTANCE_MODE);
  VL53L1X_SetTimingBudgetInMs(TOF_DEV, TOF_TIMING_BUDGET_MS);
  VL53L1X_SetInterMeasurementInMs(TOF_DEV, TOF_INTER_MEAS_MS);
  VL53L1X_StartRanging(TOF_DEV);
  s_tof_present = true;
  printf("ToF: VL53L1X ready at 0x%02X (mode=%d, timing_budget=%dms, inter_measurement=%dms)\n",
         TOF_I2C_ADDR, TOF_DISTANCE_MODE, TOF_TIMING_BUDGET_MS, TOF_INTER_MEAS_MS);
}

// ---- Sample read (returns physical units) ----------------------------------

static esp_err_t read_sample(imu_device_t *d, float accel[3], float gyro[3]) {
  uint8_t buf[14];
  int16_t rax, ray, raz, rgx, rgy, rgz;
  esp_err_t err;

  if (d->chip == CHIP_MPU6050) {
    // 0x3B..: AccelX,Y,Z, Temp, GyroX,Y,Z
    err = reg_read_bytes(d->dev, 0x3B, buf, 14);
    rax = (int16_t)((buf[0] << 8) | buf[1]);
    ray = (int16_t)((buf[2] << 8) | buf[3]);
    raz = (int16_t)((buf[4] << 8) | buf[5]);
    rgx = (int16_t)((buf[8] << 8) | buf[9]);
    rgy = (int16_t)((buf[10] << 8) | buf[11]);
    rgz = (int16_t)((buf[12] << 8) | buf[13]);
  } else {
    // 0x2D..: AccelX,Y,Z, GyroX,Y,Z, Temp
    err = reg_read_bytes(d->dev, 0x2D, buf, 14);
    rax = (int16_t)((buf[0] << 8) | buf[1]);
    ray = (int16_t)((buf[2] << 8) | buf[3]);
    raz = (int16_t)((buf[4] << 8) | buf[5]);
    rgx = (int16_t)((buf[6] << 8) | buf[7]);
    rgy = (int16_t)((buf[8] << 8) | buf[9]);
    rgz = (int16_t)((buf[10] << 8) | buf[11]);
  }

  accel[0] = (rax / ACCEL_LSB_PER_G) * G_MPS2;
  accel[1] = (ray / ACCEL_LSB_PER_G) * G_MPS2;
  accel[2] = (raz / ACCEL_LSB_PER_G) * G_MPS2;
  gyro[0] = (rgx / GYRO_LSB_PER_DPS) * DEG2RAD;
  gyro[1] = (rgy / GYRO_LSB_PER_DPS) * DEG2RAD;
  gyro[2] = (rgz / GYRO_LSB_PER_DPS) * DEG2RAD;
  return err;
}

// ---- UART console helpers ---------------------------------------------------

static void uart_console_init(void) {
  uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

static int try_read_char(void) {
  uint8_t c;
  int n = uart_read_bytes(UART_NUM_0, &c, 1, 0); // non-blocking poll
  return n == 1 ? c : -1;
}

// ---- Test 1: at-rest noise/bias ---------------------------------------------

static void run_noise_bias_test_one(imu_device_t *d) {
  const int N = 1000;
  double sumA[3] = { 0 }, sumSqA[3] = { 0 };
  double sumG[3] = { 0 }, sumSqG[3] = { 0 };

  printf("--- %s at 0x%02X (1000 samples, ~10s) ---\n", chip_name(d->chip), d->addr);
  for (int i = 0; i < N; i++) {
    float accel[3], gyro[3];
    read_sample(d, accel, gyro);
    for (int a = 0; a < 3; a++) {
      sumA[a] += accel[a];
      sumSqA[a] += (double)accel[a] * accel[a];
      sumG[a] += gyro[a];
      sumSqG[a] += (double)gyro[a] * gyro[a];
    }
    if (i % 100 == 0) { printf("."); fflush(stdout); }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  printf("\n");

  const char *axis[3] = { "X", "Y", "Z" };
  printf("Accel bias/noise (m/s^2):\n");
  for (int a = 0; a < 3; a++) {
    double mean = sumA[a] / N;
    double var = sumSqA[a] / N - mean * mean;
    printf("  %s: mean=%.5f  stddev=%.5f\n", axis[a], mean, sqrt(var));
  }
  printf("Gyro bias/noise (rad/s):\n");
  for (int a = 0; a < 3; a++) {
    double mean = sumG[a] / N;
    double var = sumSqG[a] / N - mean * mean;
    printf("  %s: mean=%.6f  stddev=%.6f\n", axis[a], mean, sqrt(var));
  }
}

static void run_noise_bias_test_all(void) {
  printf("=== Test 1: at-rest noise/bias (%d chip%s detected) ===\n",
         s_device_count, s_device_count == 1 ? "" : "s");
  for (int i = 0; i < s_device_count; i++) run_noise_bias_test_one(&s_devices[i]);
}

// ---- Test 2: axis/sign mapping (live plotter stream) ------------------------

static void run_axis_mapping_test(void) {
  imu_device_t *d = &s_devices[0];
  printf("=== Test 2: axis mapping (%s at 0x%02X) - streaming, send 's' to stop ===\n",
         chip_name(d->chip), d->addr);
  printf("Watch the plotter/monitor. At rest, one Accel axis should read ~+9.81.\n");
  printf("Prop one edge up on a book and note which Gyro axis moves, and its sign.\n");

  while (true) {
    int c = try_read_char();
    if (c == 's') break;
    float accel[3], gyro[3];
    read_sample(d, accel, gyro);
    printf("AccelX:%.4f,AccelY:%.4f,AccelZ:%.4f,GyroX:%.4f,GyroY:%.4f,GyroZ:%.4f\n",
           accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
    vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz, readable on a plotter without flooding
  }
  printf("=== Test 2 stopped ===\n");
}

// ---- Test 3: achievable loop rate --------------------------------------------

#define LOOP_RATE_TARGET_HZ    1000
#define LOOP_RATE_PERIOD_US    (1000000 / LOOP_RATE_TARGET_HZ)

static void run_loop_rate_test(void) {
  imu_device_t *d = &s_devices[1];
  const int N = 1000;
  printf("=== Test 3: loop rate at a paced %dHz target (%s at 0x%02X, 1000 reads) ===\n",
         LOOP_RATE_TARGET_HZ, chip_name(d->chip), d->addr);

  double sumDt = 0, sumDtSq = 0;
  int error_count = 0;
  int64_t prev = 0;
  int64_t iter_start = esp_timer_get_time();
  for (int i = 0; i < N; i++) {
    float accel[3], gyro[3];
    if (read_sample(d, accel, gyro) != ESP_OK) error_count++; // no printing in the loop - would skew timing

    // Pace to LOOP_RATE_PERIOD_US regardless of how long the read itself took.
    int64_t target = iter_start + LOOP_RATE_PERIOD_US;
    int64_t now = esp_timer_get_time();
    if (now < target) esp_rom_delay_us((uint32_t)(target - now));
    now = esp_timer_get_time();
    iter_start = now;

    if (i > 0) {
      double dt = (now - prev) / 1e6;
      sumDt += dt;
      sumDtSq += dt * dt;
    }
    prev = now;
  }

  double achievedHz = (N - 1) / sumDt;
  double meanDt = sumDt / (N - 1);
  double varDt = sumDtSq / (N - 1) - meanDt * meanDt;
  double jitterUs = sqrt(varDt) * 1e6;

  printf("Achieved rate: %.1f Hz (target %dHz)\n", achievedHz, LOOP_RATE_TARGET_HZ);
  printf("Mean interval: %.3f ms\n", meanDt * 1000.0);
  printf("Jitter (stddev of interval): %.1f us\n", jitterUs);
  printf("I2C errors: %d / %d reads\n", error_count, N);
}

// ---- Test 4: ToF rate + no-new-data signaling -------------------------------

#define TOF_RATE_TEST_DURATION_MS 3000

static void run_tof_rate_test(void) {
  if (!s_tof_present) { printf("ToF sensor not initialized -- check wiring and reset.\n"); return; }

  printf("=== Test 4: ToF rate + no-new-data signaling (~%dms @ 100Hz poll) ===\n",
         TOF_RATE_TEST_DURATION_MS);
  vl53l1_platform_reset_error_count();

  int total_polls = 0, ready_polls = 0;
  int64_t last_ready_ts = -1;
  double sum_period = 0;
  int period_count = 0;

  int64_t test_start = esp_timer_get_time();
  int64_t iter_start = test_start;
  while (esp_timer_get_time() - test_start < TOF_RATE_TEST_DURATION_MS * 1000) {
    uint8_t ready = 0;
    VL53L1X_CheckForDataReady(TOF_DEV, &ready);
    total_polls++;
    if (ready) {
      ready_polls++;
      int64_t now = esp_timer_get_time();
      if (last_ready_ts >= 0) {
        sum_period += (now - last_ready_ts) / 1e6;
        period_count++;
      }
      last_ready_ts = now;
      uint16_t distance_mm;
      uint8_t range_status;
      VL53L1X_GetDistance(TOF_DEV, &distance_mm);
      VL53L1X_GetRangeStatus(TOF_DEV, &range_status);
      VL53L1X_ClearInterrupt(TOF_DEV); // acknowledge -> sensor auto-restarts (timed mode)
      printf("  ready: distance=%umm status=%u\n", distance_mm, range_status);
    }
    int64_t target = iter_start + MAIN_LOOP_PERIOD_US;
    int64_t now = esp_timer_get_time();
    if (now < target) esp_rom_delay_us((uint32_t)(target - now));
    iter_start = esp_timer_get_time();
  }

  printf("Total 100Hz polls: %d -- HAL_OK: %d (%.1f%%), HAL_NO_NEW_DATA: %d (%.1f%%)\n",
         total_polls, ready_polls, 100.0 * ready_polls / total_polls,
         total_polls - ready_polls, 100.0 * (total_polls - ready_polls) / total_polls);
  if (period_count > 0) {
    printf("Achieved ToF sample rate: %.2f Hz (mean inter-sample period %.1fms, configured for ~%.0fHz)\n",
           period_count / sum_period, 1000.0 * sum_period / period_count, 1000.0 / TOF_INTER_MEAS_MS);
  } else {
    printf("No ready samples seen -- check wiring/target in front of sensor.\n");
  }
  printf("ToF I2C errors: %lu\n", (unsigned long)vl53l1_platform_get_error_count());
}

// ---- Test 5: ToF noise across range ------------------------------------------

#define TOF_NOISE_SAMPLES_PER_BATCH 50

static void run_tof_noise_across_range_test(void) {
  if (!s_tof_present) { printf("ToF sensor not initialized -- check wiring and reset.\n"); return; }

  printf("=== Test 5: ToF noise across range ===\n");
  printf("Position a flat target at a known distance (measure it yourself), then press\n");
  printf("any key to sample %d readings there. Press 's' to stop this test.\n", TOF_NOISE_SAMPLES_PER_BATCH);

  while (true) {
    int c = -1;
    while (c < 0) {
      c = try_read_char();
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (c == 's') break;

    vl53l1_platform_reset_error_count();
    double sum = 0, sumSq = 0;
    int valid = 0, invalid = 0;
    for (int i = 0; i < TOF_NOISE_SAMPLES_PER_BATCH; i++) {
      // Paced at the same 100Hz cadence as Tests 4/6, proven safe on this
      // hardware/ESP-IDF combo -- a naive 5ms vTaskDelay loop here produced
      // repeated "I2C software timeout" errors (see hardware_bringup/README.md),
      // the same back-to-back-transaction failure mode Test 3 first hit.
      uint8_t ready = 0;
      int64_t poll_start = esp_timer_get_time();
      while (!ready) {
        VL53L1X_CheckForDataReady(TOF_DEV, &ready);
        if (ready) break;
        int64_t target = poll_start + MAIN_LOOP_PERIOD_US;
        int64_t now = esp_timer_get_time();
        if (now < target) esp_rom_delay_us((uint32_t)(target - now));
        poll_start = esp_timer_get_time();
      }
      uint16_t distance_mm;
      uint8_t range_status;
      VL53L1X_GetDistance(TOF_DEV, &distance_mm);
      VL53L1X_GetRangeStatus(TOF_DEV, &range_status);
      VL53L1X_ClearInterrupt(TOF_DEV);
      if (range_status == 0) { // VL53L1_RANGESTATUS_RANGE_VALID
        sum += distance_mm;
        sumSq += (double)distance_mm * distance_mm;
        valid++;
      } else {
        invalid++;
      }
    }
    if (valid > 0) {
      double mean = sum / valid;
      double var = sumSq / valid - mean * mean;
      printf("  %d/%d valid: mean=%.1fmm stddev=%.2fmm (%d non-valid-status samples, %lu I2C errors)\n",
             valid, TOF_NOISE_SAMPLES_PER_BATCH, mean, sqrt(var), invalid,
             (unsigned long)vl53l1_platform_get_error_count());
    } else {
      printf("  0/%d valid (all non-zero range_status) -- check target/alignment.\n",
             TOF_NOISE_SAMPLES_PER_BATCH);
    }
    printf("Reposition target and press a key to sample again, or 's' to stop.\n");
  }
  printf("=== Test 5 stopped ===\n");
}

// ---- Test 6: combined I2C bus timing (IMU + ToF sharing one bus) -----------

#define COMBINED_TEST_N 1000

static void run_combined_bus_timing_test(void) {
  if (s_device_count == 0) { printf("No IMU detected.\n"); return; }
  if (!s_tof_present) { printf("ToF sensor not initialized -- check wiring and reset.\n"); return; }

  imu_device_t *imu = &s_devices[0];
  printf("=== Test 6: combined I2C bus timing, IMU (%s) + ToF, paced 100Hz, %d ticks ===\n",
         chip_name(imu->chip), COMBINED_TEST_N);
  vl53l1_platform_reset_error_count();

  int imu_errors = 0, tof_ready_count = 0;
  double sumDt = 0, sumDtSq = 0;
  int64_t prev = 0;
  int64_t iter_start = esp_timer_get_time();
  for (int i = 0; i < COMBINED_TEST_N; i++) {
    float accel[3], gyro[3];
    if (read_sample(imu, accel, gyro) != ESP_OK) imu_errors++;

    uint8_t ready = 0;
    VL53L1X_CheckForDataReady(TOF_DEV, &ready);
    if (ready) {
      uint16_t distance_mm;
      uint8_t range_status;
      VL53L1X_GetDistance(TOF_DEV, &distance_mm);
      VL53L1X_GetRangeStatus(TOF_DEV, &range_status);
      VL53L1X_ClearInterrupt(TOF_DEV);
      tof_ready_count++;
    }

    int64_t target = iter_start + MAIN_LOOP_PERIOD_US;
    int64_t now = esp_timer_get_time();
    if (now < target) esp_rom_delay_us((uint32_t)(target - now));
    now = esp_timer_get_time();
    iter_start = now;
    if (i > 0) {
      double dt = (now - prev) / 1e6;
      sumDt += dt;
      sumDtSq += dt * dt;
    }
    prev = now;
  }

  double achievedHz = (COMBINED_TEST_N - 1) / sumDt;
  double meanDt = sumDt / (COMBINED_TEST_N - 1);
  double varDt = sumDtSq / (COMBINED_TEST_N - 1) - meanDt * meanDt;
  double expected_tof_pct = 100.0 * (1000.0 / TOF_INTER_MEAS_MS) / 100.0;

  printf("Achieved combined-loop rate: %.1f Hz (target 100Hz)\n", achievedHz);
  printf("Mean interval: %.3fms, Jitter (stddev): %.1fus\n", meanDt * 1000.0, sqrt(varDt) * 1e6);
  printf("IMU I2C errors: %d / %d\n", imu_errors, COMBINED_TEST_N);
  printf("ToF data-ready count: %d / %d ticks (%.1f%%, expect ~%.1f%% at 100Hz poll / %.0fHz ToF rate)\n",
         tof_ready_count, COMBINED_TEST_N, 100.0 * tof_ready_count / COMBINED_TEST_N, expected_tof_pct,
         1000.0 / TOF_INTER_MEAS_MS);
  printf("ToF I2C errors: %lu\n", (unsigned long)vl53l1_platform_get_error_count());
}

// ---- Menu / main ----------------------------------------------------------

static void print_menu(void) {
  printf("\n--- IMU/ToF bring-up menu ---\n");
  printf("1: IMU noise/bias   2: IMU axis mapping   3: IMU loop rate\n");
  printf("4: ToF rate + no-new-data   5: ToF noise across range   6: combined I2C bus timing\n");
}

void app_main(void) {
  uart_console_init();
  i2c_bus_init();

  printf("Detecting IMU(s)...\n");
  detect_devices();
  if (s_device_count == 0) {
    printf("ERROR: no known IMU found at 0x68/0x69. Check wiring and reset.\n");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  for (int i = 0; i < s_device_count; i++) {
    init_device(&s_devices[i]);
    printf("Detected: %s at 0x%02X\n", chip_name(s_devices[i].chip), s_devices[i].addr);
  }
  if (s_device_count > 1) {
    printf("(Tests 2/3 run against the first device listed above: %s at 0x%02X)\n",
           chip_name(s_devices[0].chip), s_devices[0].addr);
  }

  printf("Initializing ToF sensor...\n");
  tof_init();

  print_menu();

  while (true) {
    int c = try_read_char();
    if (c == '1') { run_noise_bias_test_all(); print_menu(); }
    else if (c == '2') { run_axis_mapping_test(); print_menu(); }
    else if (c == '3') { run_loop_rate_test(); print_menu(); }
    else if (c == '4') { run_tof_rate_test(); print_menu(); }
    else if (c == '5') { run_tof_noise_across_range_test(); print_menu(); }
    else if (c == '6') { run_combined_bus_timing_test(); print_menu(); }
    else vTaskDelay(pdMS_TO_TICKS(10)); // avoid busy-spin while idle
  }
}
