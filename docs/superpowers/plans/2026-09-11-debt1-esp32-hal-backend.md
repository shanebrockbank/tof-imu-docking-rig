# DEBT-1: ESP32 HAL Backend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the real ESP32 HAL backend (`hal_esp32.c`) and its ESP-IDF app entry point (`main_esp32.c`), so the existing pure-C99 estimation/guidance/logging pipeline runs against real ICM20948/VL53L1X/servo hardware instead of the host simulator.

**Architecture:** A new, separate ESP-IDF project (`firmware/`) reuses the existing `common/`, `estimation/`, `guidance/`, `logging/` sources unchanged, adds `hal/hal_esp32.c` (the sole ESP-IDF-including file) implementing the same `hal_t` interface `hal_host.c` implements, and adds a `hal_t.pace_tick` member so `main_esp32.c` and `main_host_sim.c` share an identical loop body — only the injected `hal_t` differs. The VL53L1X ToF driver reuses the vendored ULD + platform-shim already proven in `hardware_bringup`; the ICM20948 IMU driver reuses that firmware's proven register sequence, locked to ICM20948 only (no MPU6050 support). Axis-mapping and servo constants are explicitly-flagged placeholders pending physical bring-up, per the approved design spec.

**Tech Stack:** C99 (host + ESP32), ESP-IDF v5.5.2 (`~/esp/esp-idf`, `source ~/esp/esp-idf/export.sh` once per shell), CMake/CTest (host suite), idf.py/ninja (ESP32 firmware build).

**Spec:** `docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md`

## Global Constraints

- Only `hal/hal_esp32.c` may `#include` any ESP-IDF header (`driver/*.h`, `esp_*.h`, `freertos/*.h`, `sdkconfig.h`); `hal_esp32.h` re-exports none of them. Mechanically checked by `tests/test_hal_boundary.sh` — must stay green after every task.
- `common/`, `estimation/`, `guidance/` stay pure C99 with zero hardware deps; this plan does not modify any file in those directories.
- No dynamic allocation after init — all new state (I2C handles, LEDC config, pacing counters) is file-static, fixed-size.
- Never call estimation/guidance/control code from ISR context — this backend is fully polled, no ISR/interrupt-pin usage (matches V1's design).
- Any two same-concept-different-values status codes get an explicit translator function, never a raw cast (`vl53l1x_translate_status`, `translate_esp_err_to_hal_status`).
- Run the full host test suite after every task and report the pass count; a task is not done until that gate is green (in addition to each task's own `idf.py build` check, once `firmware/` exists).
- Axis-mapping and servo constants are deliberate, explicitly-commented placeholders (per the approved spec §1.1/§8) — do not attempt to "fix" them with guessed real values; that only happens after physical bring-up, outside this plan.
- Do not fold DEBT-2/3/4 scope into this work.

---

## Task 1: `hal_t.pace_tick` — interface change + host no-op

**Files:**
- Modify: `hal/hal.h`
- Modify: `hal/hal_host.c`
- Modify: `main_host_sim.c`
- Test: `tests/test_hal_host.c`

**Interfaces:**
- Produces: `hal_t.pace_tick` — `void (*pace_tick)(void *ctx)`, a new member on the existing `hal_t` struct. Every backend's `_create()` must populate it. Called once per main-loop iteration by both mains.

- [ ] **Step 1: Write the failing test**

Add this test to `tests/test_hal_host.c`, right after `test_clock_now_reflects_ticks`:

```c
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
```

And add `RUN_TEST(test_pace_tick_is_noop);` to `main()`, so the full file's `main()` reads:

```c
int main(void) {
    RUN_TEST(test_hal_t_reads_route_through_world_state);
    RUN_TEST(test_actuator_command_is_captured);
    RUN_TEST(test_clock_now_reflects_ticks);
    RUN_TEST(test_tick_advances_true_cart_state);
    RUN_TEST(test_pace_tick_is_noop);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake -S . -B build && cmake --build build --target test_hal_host`
Expected: FAIL to compile — `hal_t` has no member named `pace_tick`.

- [ ] **Step 3: Add `pace_tick` to `hal_t`**

In `hal/hal.h`, add a new member after `clock_now` (immediately before the closing `} hal_t;`):

```c
    /* Blocks until the next main-loop tick boundary (real backends); a
       no-op on backends with no wall clock to wait on (e.g. host/sim).
       Called once per loop iteration by both main_host_sim.c and
       main_esp32.c so their loop bodies are identical in shape — see
       docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md §4. */
    void (*pace_tick)(void *ctx);
```

- [ ] **Step 4: Implement the host no-op and wire it in**

In `hal/hal_host.c`, add this static function right after `host_clock_now`:

```c
static void host_pace_tick(void *ctx) {
    (void)ctx; /* host/sim has no wall clock to wait on; time advances via hal_host_world_tick() */
}
```

Then in `hal_host_create()`, add one line so the full function reads:

```c
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
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build --target test_hal_host && ctest --test-dir build -R test_hal_host --output-on-failure`
Expected: PASS, `5/5 tests passing`.

- [ ] **Step 6: Wire `pace_tick` into `main_host_sim.c`'s loop**

In `main_host_sim.c`, inside the `for (int i = 0; i < N_TICKS; i++)` loop, insert one line right after the existing `hal_host_world_tick(...)` call and before `timestamp_t now = h.clock_now(h.ctx);`:

```c
        hal_host_world_tick(&world, true_accel_mps2, 0.0, TICK_DT);
        h.pace_tick(h.ctx);
        timestamp_t now = h.clock_now(h.ctx);
```

This is a no-op call on this backend (`hal_host_world_tick` is what actually advances simulated time) — it exists purely so the loop body's shape matches `main_esp32.c`'s.

- [ ] **Step 7: Run the full host suite and confirm the demo runner still builds**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass (report the exact "N/N tests passing" count). `main_host_sim` target must also build without error.

- [ ] **Step 8: Commit**

```bash
git add hal/hal.h hal/hal_host.c main_host_sim.c tests/test_hal_host.c
git commit -m "$(cat <<'EOF'
add: hal_t.pace_tick member for host/ESP32 main-loop parity

Adds a pace_tick(ctx) member to hal_t so main_host_sim.c and the
upcoming main_esp32.c share an identical loop body — only the injected
hal_t backend differs. Host backend implements it as a no-op (time
already advances via hal_host_world_tick()).
EOF
)"
```

---

## Task 2: `vl53l1x_status` translator (pure C99, host-testable)

**Files:**
- Create: `hal/vl53l1x_status.h`
- Create: `hal/vl53l1x_status.c`
- Test: `tests/test_vl53l1x_status.c`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `hal_status_t` (`common/types.h`, already defined: `HAL_OK`, `HAL_NO_NEW_DATA`, `HAL_FAULT`).
- Produces: `hal_status_t vl53l1x_translate_status(bool data_ready, uint8_t range_status)` — used by `hal_esp32.c`'s ToF backend in Task 7.

- [ ] **Step 1: Write the failing test**

Create `tests/test_vl53l1x_status.c`:

```c
#include "tests/test_framework.h"
#include "hal/vl53l1x_status.h"

TEST(test_not_ready_is_no_new_data) {
    CHECK(vl53l1x_translate_status(false, 0) == HAL_NO_NEW_DATA);
    CHECK(vl53l1x_translate_status(false, 7) == HAL_NO_NEW_DATA);
}

TEST(test_ready_and_valid_is_ok) {
    CHECK(vl53l1x_translate_status(true, 0) == HAL_OK);
}

TEST(test_ready_and_nonzero_status_is_fault) {
    CHECK(vl53l1x_translate_status(true, 1) == HAL_FAULT);
    CHECK(vl53l1x_translate_status(true, 255) == HAL_FAULT);
}

int main(void) {
    RUN_TEST(test_not_ready_is_no_new_data);
    RUN_TEST(test_ready_and_valid_is_ok);
    RUN_TEST(test_ready_and_nonzero_status_is_fault);
    TEST_SUMMARY();
    return 0;
}
```

Add to `tests/CMakeLists.txt` (append at the end, after the `test_hal_boundary` line):

```cmake
add_library(vl53l1x_status_lib STATIC ${CMAKE_SOURCE_DIR}/hal/vl53l1x_status.c)

add_executable(test_vl53l1x_status test_vl53l1x_status.c)
target_link_libraries(test_vl53l1x_status vl53l1x_status_lib)
add_test(NAME test_vl53l1x_status COMMAND test_vl53l1x_status)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake -S . -B build && cmake --build build --target test_vl53l1x_status`
Expected: FAIL — `hal/vl53l1x_status.h` does not exist yet.

- [ ] **Step 3: Write the header**

Create `hal/vl53l1x_status.h`:

```c
#ifndef HAL_VL53L1X_STATUS_H
#define HAL_VL53L1X_STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include "common/types.h"

/* Translates the VL53L1X ULD's own (data-ready flag, range_status byte)
   pair into this project's hal_status_t — an explicit translator per
   CLAUDE.md constraint 7 (never a raw cast between same-concept,
   different-value status codes). range_status == 0 is the ULD's own
   VL53L1_RANGESTATUS_RANGE_VALID; any other value is a fault (out of
   range, signal failure, etc. — see ST's VL53L1X_error_codes.h). */
hal_status_t vl53l1x_translate_status(bool data_ready, uint8_t range_status);

#endif /* HAL_VL53L1X_STATUS_H */
```

- [ ] **Step 4: Write the implementation**

Create `hal/vl53l1x_status.c`:

```c
#include "hal/vl53l1x_status.h"

hal_status_t vl53l1x_translate_status(bool data_ready, uint8_t range_status) {
    if (!data_ready) return HAL_NO_NEW_DATA;
    return (range_status == 0) ? HAL_OK : HAL_FAULT;
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build --target test_vl53l1x_status && ctest --test-dir build -R test_vl53l1x_status --output-on-failure`
Expected: PASS, `3/3 tests passing`.

- [ ] **Step 6: Run the full host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count.

- [ ] **Step 7: Commit**

```bash
git add hal/vl53l1x_status.h hal/vl53l1x_status.c tests/test_vl53l1x_status.c tests/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: VL53L1X status translator (pure C99, host-testable)

Explicit translator from the VL53L1X ULD's (data_ready, range_status)
pair to hal_status_t, per CLAUDE.md constraint 7. Lives under hal/ but
has no ESP-IDF dependency, so it's host-unit-testable even though the
ToF backend that will use it (hal_esp32.c) is not.
EOF
)"
```

---

## Task 3: `csv_logger` — stream-based open + hardware row variant

**Files:**
- Modify: `logging/csv_logger.h`
- Modify: `logging/csv_logger.c`
- Modify: `tests/test_csv_logger.c`

**Interfaces:**
- Produces: `bool csv_logger_open_stream(csv_logger_t *log, FILE *stream)`, `void csv_logger_write_row_hw(csv_logger_t *log, double t_s, double measured_range_m, double raw_speed_mps, bool raw_is_stale, double fused_speed_mps, double v_safe_mps, double speed_error_mps, double control_output_unfiltered, double control_output_filtered, double servo_angle_deg)` — used by `main_esp32.c` in Task 9.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_csv_logger.c` (after the existing two `TEST` blocks, before `int main(void)`), and add `#include <stdio.h>` is already present via `csv_logger.h`:

```c
TEST(test_open_stream_writes_hw_header_row) {
    csv_logger_t log;
    FILE *f = tmpfile();
    CHECK(f != NULL);
    CHECK(csv_logger_open_stream(&log, f));
    fflush(log.fp);
    rewind(f);
    char line[512];
    fgets(line, sizeof(line), f);
    CHECK(strstr(line, "measured_range_m") != NULL);
    CHECK(strstr(line, "true_range_m") == NULL);
    fclose(f);
}

TEST(test_write_row_hw_produces_expected_column_count) {
    csv_logger_t log;
    FILE *f = tmpfile();
    CHECK(f != NULL);
    CHECK(csv_logger_open_stream(&log, f));
    csv_logger_write_row_hw(&log, 0.01, 0.9, 0.1, false, 0.1, 0.5, -0.4, -0.6, -0.5, 60.0);
    fflush(log.fp);
    rewind(f);
    char header[512], row[512];
    fgets(header, sizeof(header), f);
    fgets(row, sizeof(row), f);
    int commas = 0;
    for (char *p = row; *p; p++) if (*p == ',') commas++;
    CHECK(commas == 9); /* 10 columns -> 9 commas */
    fclose(f);
}
```

Update `main()` at the bottom of `tests/test_csv_logger.c` to:

```c
int main(void) {
    RUN_TEST(test_open_writes_header_row);
    RUN_TEST(test_write_row_produces_expected_column_count);
    RUN_TEST(test_open_stream_writes_hw_header_row);
    RUN_TEST(test_write_row_hw_produces_expected_column_count);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target test_csv_logger`
Expected: FAIL to compile — `csv_logger_open_stream`/`csv_logger_write_row_hw` are undeclared.

- [ ] **Step 3: Add the declarations**

In `logging/csv_logger.h`, add after the existing `csv_logger_write_row` declaration and before `csv_logger_close`:

```c
/* Points log->fp at an already-open stream (e.g. stdout on ESP32, where
   there is no mounted filesystem to fopen() a path against) and writes
   the hardware-variant header row. Always succeeds for a valid stream;
   returns bool for symmetry with csv_logger_open(). */
bool csv_logger_open_stream(csv_logger_t *log, FILE *stream);

/* Hardware-variant row: same columns as csv_logger_write_row() minus
   true_range_m/true_velocity_mps (simulation ground truth only, per
   docs/design.md §8 — no hardware analog). measured_range_m is the real
   last-known ToF reading, in true_range_m's old column position. */
void csv_logger_write_row_hw(csv_logger_t *log,
                              double t_s,
                              double measured_range_m,
                              double raw_speed_mps,
                              bool raw_is_stale,
                              double fused_speed_mps,
                              double v_safe_mps,
                              double speed_error_mps,
                              double control_output_unfiltered,
                              double control_output_filtered,
                              double servo_angle_deg);
```

- [ ] **Step 4: Implement them**

In `logging/csv_logger.c`, add after `csv_logger_write_row` and before `csv_logger_close`:

```c
bool csv_logger_open_stream(csv_logger_t *log, FILE *stream) {
    log->fp = stream;
    fprintf(log->fp,
        "t_s,measured_range_m,raw_speed_mps,raw_is_stale,"
        "fused_speed_mps,v_safe_mps,speed_error_mps,"
        "control_output_unfiltered,control_output_filtered,servo_angle_deg\n");
    return true;
}

void csv_logger_write_row_hw(csv_logger_t *log,
                              double t_s,
                              double measured_range_m,
                              double raw_speed_mps,
                              bool raw_is_stale,
                              double fused_speed_mps,
                              double v_safe_mps,
                              double speed_error_mps,
                              double control_output_unfiltered,
                              double control_output_filtered,
                              double servo_angle_deg) {
    if (!log->fp) return;
    fprintf(log->fp, "%f,%f,%f,%d,%f,%f,%f,%f,%f,%f\n",
        t_s, measured_range_m, raw_speed_mps, raw_is_stale ? 1 : 0,
        fused_speed_mps, v_safe_mps, speed_error_mps,
        control_output_unfiltered, control_output_filtered, servo_angle_deg);
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build --target test_csv_logger && ctest --test-dir build -R test_csv_logger --output-on-failure`
Expected: PASS, `4/4 tests passing`.

- [ ] **Step 6: Run the full host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count.

- [ ] **Step 7: Commit**

```bash
git add logging/csv_logger.h logging/csv_logger.c tests/test_csv_logger.c
git commit -m "$(cat <<'EOF'
add: csv_logger stream-based open + hardware row variant

csv_logger_open_stream() lets a caller point the logger at an
already-open FILE* (e.g. stdout) instead of fopen()-ing a path, since
ESP32 has no mounted filesystem in scope. csv_logger_write_row_hw()
drops the two sim-ground-truth-only columns (true_range_m/
true_velocity_mps) that have no hardware analog.
EOF
)"
```

---

## Task 4: `firmware/` ESP-IDF project scaffold + vendored ToF component + compiling stub

**Files:**
- Create: `firmware/CMakeLists.txt`
- Create: `firmware/sdkconfig.defaults`
- Create: `firmware/.gitignore`
- Create: `firmware/main/CMakeLists.txt`
- Create: `firmware/main/main_esp32.c`
- Create: `hal/hal_esp32.h`
- Create: `hal/hal_esp32.c`
- Create: `firmware/components/vl53l1x_uld/` (copied from `hardware_bringup/imu_bringup/components/vl53l1x_uld/`)

**Interfaces:**
- Consumes: `hal_t` (`hal/hal.h`, Task 1's version — has `pace_tick`).
- Produces: `hal_t hal_esp32_create(void)` (`hal/hal_esp32.h`) — stub bodies only in this task; every later task replaces one stub function with a real implementation. `void app_main(void)` (`firmware/main/main_esp32.c`) — minimal, calls `pace_tick`/`clock_now` in a loop, proves the whole toolchain/project wiring before real pipeline logic is added.

- [ ] **Step 1: Copy the vendored VL53L1X ULD component**

```bash
mkdir -p firmware/components/vl53l1x_uld/include
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/CMakeLists.txt firmware/components/vl53l1x_uld/
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/README.md firmware/components/vl53l1x_uld/
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/VL53L1X_api.c firmware/components/vl53l1x_uld/
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/VL53L1X_calibration.c firmware/components/vl53l1x_uld/
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/vl53l1_platform.c firmware/components/vl53l1x_uld/
cp hardware_bringup/imu_bringup/components/vl53l1x_uld/include/*.h firmware/components/vl53l1x_uld/include/
diff -rq hardware_bringup/imu_bringup/components/vl53l1x_uld firmware/components/vl53l1x_uld
```

Expected: `diff -rq` prints nothing (identical copy). This is deliberate duplication — see spec §8; `hardware_bringup/` keeps its own copy until that directory is deleted.

- [ ] **Step 2: Create the ESP-IDF project root files**

Create `firmware/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tof_imu_docking_rig_esp32)
```

Create `firmware/sdkconfig.defaults`:

```
CONFIG_IDF_TARGET="esp32"
```

Create `firmware/.gitignore`:

```
build/
sdkconfig
managed_components/
dependencies.lock
```

- [ ] **Step 3: Write the `hal_esp32.h` interface header**

Create `hal/hal_esp32.h`:

```c
#ifndef HAL_HAL_ESP32_H
#define HAL_HAL_ESP32_H

#include "hal/hal.h"

/* Populates and returns a hal_t bound to the real ESP32 peripherals (I2C
   bus, ICM20948, VL53L1X, servo PWM, esp_timer clock). Zero ESP-IDF types
   appear in this header — see CLAUDE.md constraint 1. Call once at
   startup, from app_main() in main_esp32.c. Backend state is file-static
   inside hal_esp32.c (there is exactly one board), so this takes no
   arguments, unlike hal_host_create(). */
hal_t hal_esp32_create(void);

#endif /* HAL_HAL_ESP32_H */
```

- [ ] **Step 4: Write the `hal_esp32.c` compiling stub**

Create `hal/hal_esp32.c`:

```c
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
```

- [ ] **Step 5: Write the minimal `main_esp32.c`**

Create `firmware/main/main_esp32.c`:

```c
#include "hal/hal_esp32.h"

void app_main(void) {
    hal_t h = hal_esp32_create();
    for (;;) {
        h.pace_tick(h.ctx);
        (void)h.clock_now(h.ctx);
    }
}
```

- [ ] **Step 6: Write the `main` component's CMakeLists.txt**

Create `firmware/main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c"
    INCLUDE_DIRS "../.."
    PRIV_REQUIRES esp_timer
)
```

- [ ] **Step 7: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware
idf.py set-target esp32
idf.py build
cd ..
```

Expected: build succeeds (`Project build complete.`). This is the first real ESP-IDF compile of this project — it proves the project scaffold, component discovery, and `INCLUDE_DIRS ../..` repo-root-relative include convention all work, before any real hardware logic is added.

- [ ] **Step 8: Run the host suite (must still be unaffected)**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count. `tests/test_hal_boundary.sh` must still pass (`hal_esp32.c` is the expected sole exception).

- [ ] **Step 9: Commit**

```bash
git add firmware/ hal/hal_esp32.h hal/hal_esp32.c
git commit -m "$(cat <<'EOF'
add: firmware/ ESP-IDF project scaffold + hal_esp32.c stub

New, separate ESP-IDF project (firmware/) alongside the host-only root
CMakeLists.txt. hal_esp32.c/.h stub out the full hal_t interface so the
project builds end-to-end; each backend function is filled in by a
later commit. Also copies the proven vendored VL53L1X ULD component
from hardware_bringup/ (accepted duplication — see spec §8).
EOF
)"
```

---

## Task 5: `hal_esp32.c` — real clock + pacing

**Files:**
- Modify: `hal/hal_esp32.c`
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Produces: real `esp32_clock_now`/`esp32_pace_tick` bodies (same names/signatures as the Task 4 stub — no interface change).

- [ ] **Step 1: Replace the clock and pace stubs**

In `hal/hal_esp32.c`, add these includes at the top, right after the file header comment:

```c
#include "esp_timer.h"
#include "esp_rom_sys.h"
```

Add this constant near the top of the file (after the includes):

```c
#define MAIN_LOOP_PERIOD_US 10000 /* 100 Hz, docs/design.md §5.6 */

static int64_t s_pace_iter_start_us = 0;
```

Replace the `esp32_clock_now` stub body:

```c
static timestamp_t esp32_clock_now(void *ctx) {
    (void)ctx;
    timestamp_t t;
    t.t_s = (double)esp_timer_get_time() / 1e6;
    return t;
}
```

Replace the `esp32_pace_tick` stub body:

```c
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
```

In `hal_esp32_create()`, initialize the pacing baseline so the very first `pace_tick` call doesn't wait a full stale period. Add one line at the top of the function body:

```c
hal_t hal_esp32_create(void) {
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
```

- [ ] **Step 2: Add `esp_rom` to the component's requirements**

In `firmware/main/CMakeLists.txt`, update `PRIV_REQUIRES`:

```cmake
idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c"
    INCLUDE_DIRS "../.."
    PRIV_REQUIRES esp_timer esp_rom
)
```

- [ ] **Step 3: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware && idf.py build && cd ..
```

Expected: build succeeds.

- [ ] **Step 4: Run the host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count (host code is untouched by this task, so this is a regression check).

- [ ] **Step 5: Commit**

```bash
git add hal/hal_esp32.c firmware/main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: hal_esp32.c real clock_now + pace_tick

esp_timer_get_time()-based clock, and the busy-wait remainder-of-period
pacer already proven reliable on this hardware in hardware_bringup
Tests 3/6 (deliberately not vTaskDelay-based).
EOF
)"
```

---

## Task 6: `hal_esp32.c` — real servo actuator (LEDC PWM)

**Files:**
- Modify: `hal/hal_esp32.c`
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Produces: real `esp32_actuator_set_angle_deg` body (same name/signature as the Task 4 stub).

- [ ] **Step 1: Add the LEDC include and servo constants**

In `hal/hal_esp32.c`, add to the includes:

```c
#include "driver/ledc.h"
```

Add these constants (near the other `#define`s):

```c
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
```

- [ ] **Step 2: Add a servo init function**

Add this function above `esp32_actuator_set_angle_deg`:

```c
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
```

- [ ] **Step 3: Replace the actuator stub body**

```c
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
```

- [ ] **Step 4: Call `servo_init()` from `hal_esp32_create()`**

Add one line, so the function reads:

```c
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
```

- [ ] **Step 5: Add `driver` to the component's requirements**

In `firmware/main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c"
    INCLUDE_DIRS "../.."
    PRIV_REQUIRES esp_timer esp_rom driver
)
```

- [ ] **Step 6: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware && idf.py build && cd ..
```

Expected: build succeeds.

- [ ] **Step 7: Run the host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count.

- [ ] **Step 8: Commit**

```bash
git add hal/hal_esp32.c firmware/main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: hal_esp32.c real servo actuator backend (LEDC PWM)

50Hz/500-2500us hobby-servo PWM via ESP-IDF's LEDC driver. GPIO pin and
pulse-width range are explicitly-flagged placeholders (no servo picked/
wired yet) per the approved design spec — verify/tune once real
hardware exists. Bounds-checks the incoming angle per docs/design.md
§5.3, returning HAL_FAULT rather than silently clamping.
EOF
)"
```

---

## Task 7: `hal_esp32.c` — real ToF backend (VL53L1X)

**Files:**
- Modify: `hal/hal_esp32.c`
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `vl53l1x_translate_status` (`hal/vl53l1x_status.h`, Task 2), `vl53l1_platform_bind`/`VL53L1X_*` (`components/vl53l1x_uld`, Task 4's copy).
- Produces: real `esp32_range_read` body; adds a shared I2C bus (`s_i2c_bus`), reused by Task 8's IMU backend.

- [ ] **Step 1: Add includes**

In `hal/hal_esp32.c`, add:

```c
#include "driver/i2c_master.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "hal/vl53l1x_status.h"
```

- [ ] **Step 2: Add I2C bus + ToF constants and shared bus handle**

```c
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

static i2c_master_bus_handle_t s_i2c_bus;
static bool s_tof_ready = false;
```

- [ ] **Step 3: Add I2C bus init and ToF init functions**

```c
static void i2c_bus_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_config, &s_i2c_bus);
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
```

- [ ] **Step 4: Replace the `esp32_range_read` stub body**

```c
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
```

- [ ] **Step 5: Call the init functions from `hal_esp32_create()`**

```c
hal_t hal_esp32_create(void) {
    s_pace_iter_start_us = esp_timer_get_time();
    servo_init();
    i2c_bus_init();
    tof_init();
    hal_t h;
    h.ctx = NULL;
    h.range_read = esp32_range_read;
    h.imu_read = esp32_imu_read;
    h.actuator_set_angle_deg = esp32_actuator_set_angle_deg;
    h.clock_now = esp32_clock_now;
    h.pace_tick = esp32_pace_tick;
    return h;
}
```

- [ ] **Step 6: Update `firmware/main/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c" "../../hal/vl53l1x_status.c"
    INCLUDE_DIRS "../.."
    REQUIRES vl53l1x_uld
    PRIV_REQUIRES esp_timer esp_rom driver
)
```

- [ ] **Step 7: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware && idf.py build && cd ..
```

Expected: build succeeds (this is the first build that links against the vendored `vl53l1x_uld` component).

- [ ] **Step 8: Run the host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count (`hal/vl53l1x_status.c` is now included in two different builds — the host `vl53l1x_status_lib` from Task 2, and this ESP-IDF component list — with no shared build artifacts between them, so this is purely a regression check on the host side).

- [ ] **Step 9: Commit**

```bash
git add hal/hal_esp32.c firmware/main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: hal_esp32.c real ToF backend (VL53L1X)

Reuses the exact CheckForDataReady -> GetDistance/GetRangeStatus ->
ClearInterrupt sequence already proven in hardware_bringup, continuous
Timed ranging (no interrupt pin — polled-only per docs/design.md §5.5).
Status translation goes through vl53l1x_translate_status(), not a raw
cast. Distance mode / timing budget / inter-measurement period are the
measured-good values from hardware_bringup Tests 4/5, not placeholders.
EOF
)"
```

---

## Task 8: `hal_esp32.c` — real IMU backend (ICM20948)

**Files:**
- Modify: `hal/hal_esp32.c`

**Interfaces:**
- Consumes: `s_i2c_bus` (Task 7).
- Produces: real `esp32_imu_read` body. Adds `translate_esp_err_to_hal_status` (a small named translator, constraint 7).

- [ ] **Step 1: Add includes and IMU constants**

Add to the includes:

```c
#include <math.h>
```

Add constants:

```c
#define IMU_I2C_ADDR     0x69  /* ICM20948, matches current bring-up wiring (AD0 high) */
#define ACCEL_LSB_PER_G  16384.0
#define GYRO_LSB_PER_DPS 131.0
#define G_MPS2           9.81
#define DEG2RAD          (M_PI / 180.0)

static i2c_master_dev_handle_t s_imu_dev;
static bool s_imu_ready = false;
```

- [ ] **Step 2: Add register helpers and the ICM20948 init sequence**

```c
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
```

Add the required FreeRTOS include for `vTaskDelay`/`pdMS_TO_TICKS`:

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- [ ] **Step 3: Replace the `esp32_imu_read` stub body**

```c
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
```

- [ ] **Step 4: Call `imu_init()` from `hal_esp32_create()`**

```c
hal_t hal_esp32_create(void) {
    s_pace_iter_start_us = esp_timer_get_time();
    servo_init();
    i2c_bus_init();
    tof_init();
    imu_init();
    hal_t h;
    h.ctx = NULL;
    h.range_read = esp32_range_read;
    h.imu_read = esp32_imu_read;
    h.actuator_set_angle_deg = esp32_actuator_set_angle_deg;
    h.clock_now = esp32_clock_now;
    h.pace_tick = esp32_pace_tick;
    return h;
}
```

- [ ] **Step 5: Add `freertos` to the component's requirements**

`vTaskDelay`/`pdMS_TO_TICKS` need it explicitly. Update `firmware/main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c" "../../hal/vl53l1x_status.c"
    INCLUDE_DIRS "../.."
    REQUIRES vl53l1x_uld
    PRIV_REQUIRES esp_timer esp_rom driver freertos
)
```

- [ ] **Step 6: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware && idf.py build && cd ..
```

Expected: build succeeds.

- [ ] **Step 7: Run the host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count.

- [ ] **Step 8: Commit**

```bash
git add hal/hal_esp32.c firmware/main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: hal_esp32.c real IMU backend (ICM20948, locked, no MPU6050)

Reuses the exact bank-select/PWR_MGMT/register-read sequence already
proven in hardware_bringup, locked to ICM20948 only (no runtime chip
detection) per project owner decision. Axis/sign mapping is an
explicitly-flagged placeholder — hardware_bringup Test 2 has never been
run — loudly commented so it isn't forgotten. esp_err_t -> hal_status_t
goes through a named translator, not a raw cast.
EOF
)"
```

---

## Task 9: `main_esp32.c` — full pipeline wiring

**Files:**
- Modify: `firmware/main/main_esp32.c`
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `estimator_init`/`estimator_tick` (`estimation/estimator.h`), `v_safe` (`guidance/v_safe.h`), `pd_controller_init`/`pd_controller_update` (`guidance/pd_controller.h`), `actuator_map_to_servo_deg` (`guidance/actuator_mapping.h`), `dt_between` (`common/dt_validation.h`), `csv_logger_open_stream`/`csv_logger_write_row_hw` (`logging/csv_logger.h`, Task 3).

- [ ] **Step 1: Replace `main_esp32.c` with the full wiring**

```c
#include "hal/hal_esp32.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "logging/csv_logger.h"
#include "common/dt_validation.h"

void app_main(void) {
    hal_t h = hal_esp32_create();

    estimator_t estimator;
    estimator_init(&estimator, 0.90);

    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    csv_logger_t log;
    csv_logger_open_stream(&log, stdout);

    timestamp_t prev_now = h.clock_now(h.ctx);
    double last_measured_range_m = 0.0;
    bool have_measured_range = false;

    for (;;) {
        h.pace_tick(h.ctx);
        timestamp_t now = h.clock_now(h.ctx);
        double ctrl_dt = dt_between(prev_now, now);

        range_sample_t range_sample;
        imu_sample_t imu_sample;
        hal_status_t range_status = h.range_read(h.ctx, &range_sample);
        hal_status_t imu_status = h.imu_read(h.ctx, &imu_sample);
        range_sample.status = range_status;
        imu_sample.status = imu_status;

        if (range_status == HAL_OK) {
            last_measured_range_m = range_sample.range_m;
            have_measured_range = true;
        }

        estimator_output_t est = estimator_tick(&estimator, &range_sample, &imu_sample);

        if (have_measured_range) {
            double target = v_safe(last_measured_range_m);
            double speed_error = est.fused_speed_mps - target;
            pd_output_t ctrl = pd_controller_update(&pd, speed_error, ctrl_dt);
            double servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered);
            h.actuator_set_angle_deg(h.ctx, servo_deg);

            csv_logger_write_row_hw(&log,
                now.t_s,
                last_measured_range_m,
                est.raw_speed.value_mps,
                est.raw_speed.is_stale,
                est.fused_speed_mps,
                target,
                speed_error,
                ctrl.control_output_unfiltered,
                ctrl.control_output_filtered,
                servo_deg);
        }

        prev_now = now;
    }
}
```

`have_measured_range` guards against calling `v_safe()`/logging a row before the very first valid ToF sample arrives (there's no ground-truth initial range on real hardware, unlike `main_host_sim.c`'s `INITIAL_RANGE_M`).

- [ ] **Step 2: Add the pipeline sources to `firmware/main/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "main_esp32.c"
         "../../hal/hal_esp32.c"
         "../../hal/vl53l1x_status.c"
         "../../common/dt_validation.c"
         "../../common/metrics.c"
         "../../estimation/gravity_compensation.c"
         "../../estimation/orientation_1d.c"
         "../../estimation/raw_speed.c"
         "../../estimation/complementary_filter.c"
         "../../estimation/estimator.c"
         "../../guidance/v_safe.c"
         "../../guidance/pd_controller.c"
         "../../guidance/actuator_mapping.c"
         "../../logging/csv_logger.c"
    INCLUDE_DIRS "../.."
    REQUIRES vl53l1x_uld
    PRIV_REQUIRES esp_timer esp_rom driver
)
```

(`common/metrics.c` isn't called by `main_esp32.c` but is included for parity with the host build's file set and to keep the two source lists easy to diff against each other — it costs nothing at link time since nothing references its symbols in a way that would fail.)

- [ ] **Step 3: Build it**

```bash
source ~/esp/esp-idf/export.sh
cd firmware && idf.py build && cd ..
```

Expected: build succeeds — this is the first build of the complete pipeline.

- [ ] **Step 4: Run the host suite**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the "N/N tests passing" count.

- [ ] **Step 5: Commit**

```bash
git add firmware/main/main_esp32.c firmware/main/CMakeLists.txt
git commit -m "$(cat <<'EOF'
add: main_esp32.c full pipeline wiring

Mirrors main_host_sim.c's loop body exactly (pace_tick -> clock_now ->
range_read/imu_read -> estimator_tick -> v_safe/pd_controller/
actuator_mapping -> actuator_set_angle_deg -> log row), differing only
in the injected hal_t and running forever instead of a fixed tick count.
Logs via csv_logger_write_row_hw() to stdout (captured via idf.py
monitor) since there's no mounted filesystem in scope.
EOF
)"
```

---

## Task 10: Final integration check + docs update

**Files:**
- Modify: `CLAUDE.md`
- Modify: `docs/design.md`

**Interfaces:** none (documentation + verification only).

- [ ] **Step 1: Run the full host suite one more time**

Run: `cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: all tests pass — report the exact "N/N tests passing" count.

- [ ] **Step 2: Run the HAL boundary check explicitly**

Run: `./tests/test_hal_boundary.sh`
Expected: `HAL boundary OK: no ESP-IDF includes outside hal/hal_esp32.c`

- [ ] **Step 3: Run the ESP-IDF build one more time from clean**

```bash
source ~/esp/esp-idf/export.sh
cd firmware
rm -rf build
idf.py build
cd ..
```

Expected: clean build succeeds with no warnings-as-errors surfaced.

- [ ] **Step 4: Update `CLAUDE.md`'s directory-layout note**

In `CLAUDE.md`, find this paragraph (in the "Directory layout" section, after the file tree):

```
There is no `main_esp32.c` / ESP-IDF project scaffolding yet — that's
DEBT-1, V2. Don't open ESP-IDF docs, pinouts, or servo-timing datasheets
unless the current task specifically concerns `hal_esp32.c` — none of that
material is relevant to V1.
```

Replace it with:

```
`firmware/` is the ESP-IDF project (`main_esp32.c` + `hal_esp32.c`) built
by DEBT-1 — see `docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md`.
It is a separate `idf.py` build from the host CMakeLists.txt above, and is
the one place in this repo where ESP-IDF docs/pinouts/servo-timing
material is relevant. Axis-mapping (IMU) and servo GPIO/pulse-width
constants in `hal_esp32.c` are explicitly-flagged placeholders pending
physical bring-up (`hardware_bringup` Test 2, and a servo that hasn't
been picked yet) — see that file's comments before trusting real-device
output.
```

- [ ] **Step 5: Update `CLAUDE.md`'s Status section**

Find:

```
## Status

V1 complete and reviewed. V2 (DEBT-1..4, `docs/design.md` §11) not
started — requires explicit user confirmation before beginning.
```

Replace with:

```
## Status

V1 complete and reviewed. DEBT-1 (ESP32 HAL backend + servo output)
implemented — see `docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md`.
Builds cleanly (`idf.py build`) but has NOT been flashed/run on real
hardware in this work — that verification, plus correcting the
axis-mapping and servo placeholders against real bring-up results, is an
explicit follow-up for the project owner. DEBT-2..4 not started —
requires explicit user confirmation before beginning.
```

- [ ] **Step 6: Add a brief hardware-log-format note to `docs/design.md` §10**

Find the end of §10 (right before `## 11. Known debt`):

```
`true_range_m`/`true_velocity_mps` come directly from `sim_cart` (ground
truth, logged for validation, never fed to the estimator per §8).
```

Add immediately after it:

```

**Hardware variant:** the ESP32 backend (`main_esp32.c`, DEBT-1) has no
simulated ground truth, so it uses `csv_logger_write_row_hw()` instead —
identical columns minus `true_range_m`/`true_velocity_mps` (no hardware
analog), with `measured_range_m` (the real last-known ToF reading) in
`true_range_m`'s old column position.
```

- [ ] **Step 7: Commit**

```bash
git add CLAUDE.md docs/design.md
git commit -m "$(cat <<'EOF'
docs: update CLAUDE.md/design.md for DEBT-1 completion

Directory layout and Status sections now reflect that firmware/ (the
ESP32 HAL backend + app entry point) exists and builds, with the
axis-mapping/servo placeholders and lack of on-hardware verification
called out explicitly. Documents the hardware CSV log variant in
docs/design.md §10.
EOF
)"
```

---

## Self-Review Notes (completed during plan authoring)

- **Spec coverage:** every section of the approved spec (§2 file layout, §3.1-3.5 backend behaviors, §4 `hal_t` change, §5 `main_esp32.c`, §6 logging, §7 testing/gating, §8 risks/follow-ups) maps to a task above (Tasks 4/6/7/8, 1, 9, 3, 10, and the placeholder comments embedded in Tasks 6/8 respectively).
- **Placeholder scan:** the only "TODO"-shaped content in this plan is the *actual source-code comments* marking the axis-mapping and servo constants as placeholders — these are real, intentional, spec-approved artifacts of the implementation itself (§1.1/§8 of the spec), not gaps in this plan's instructions. Every task step otherwise contains complete, concrete code.
- **Type/signature consistency:** `hal_esp32_create()`, `hal_t.pace_tick`, `vl53l1x_translate_status(bool, uint8_t)`, `csv_logger_open_stream(csv_logger_t*, FILE*)`, and `csv_logger_write_row_hw(...)` are each defined once (Tasks 4, 1, 2, 3) and consumed with identical signatures in every later task that calls them (Tasks 5-9).
