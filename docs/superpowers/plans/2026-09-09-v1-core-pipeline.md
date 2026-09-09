# V1 Core Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the full host-simulatable V1 pipeline (HAL, sim, estimation
chain, guidance/control chain, CSV logging) in pure C99, with a host-native
CMake/ctest regression suite, per `docs/design.md`.

**Architecture:** `common/` (shared types + dt validation) →
`hal/hal.h` (interface contract) + `hal/hal_host.c` (sim-backed impl) ←
`sim/` (physics/noise) → `estimation/` (RAW + gravity comp + 1D pitch +
complementary filter) → `guidance/` (v_safe + PD + actuator mapping) →
`logging/` (CSV) → `main_host_sim.c` (wiring). Every module gets host-native
unit tests before the next module is built on top of it.

**Tech Stack:** C99, CMake ≥ 3.16, ctest. No external test framework
dependency — a ~40-line custom assert/registration header
(`tests/test_framework.h`) is used so there's nothing to vendor.

**Spec:** `docs/design.md` (read in full before starting; this plan argues
from it and does not repeat every rationale — only concrete values/code).

## Global Constraints

- `hal/hal_esp32.c` is the only file allowed to include ESP-IDF headers; it
  does not exist in this plan (V2/DEBT-1). `hal_esp32.h` is not created here.
- `estimation/`, `guidance/`, `common/` are pure C99, zero hardware deps.
- `hal/` contains no simulation physics; `sim/` contains no HAL-shaped
  interface code. `hal_host.c` wires the two.
- No dynamic allocation after init anywhere. Fixed-size structs.
- Main loop: 100 Hz (dt = 0.01 s nominal). ToF: 20 Hz nominal. IMU: 100 Hz
  nominal. `DT_MAX_S = 2.0`.
- Every task ends with `ctest --test-dir build --output-on-failure` run and
  the pass count reported before moving to the next task.
- Track length: 1.0 m. `v_safe`: `k=0.5, v_cap=0.6, v_floor=0.03`.
  Complementary filter: `alpha=0.90`. PD: `Kp=1.5, Kd=0.1, tau_d=0.05`.
  Actuator mapping: `center=90deg, half_range=45deg`.
- **Checkpoint after Task 3 (HAL interfaces defined, no impl yet):** stop
  and confirm the HAL interface specs with the user before writing any
  backend (`hal_host.c`) or sim code, per the user's working agreement.

---

## Task 1: Project scaffolding — CMake, common types, dt validation, test framework

**Files:**
- Create: `CMakeLists.txt`
- Create: `common/types.h`
- Create: `common/dt_validation.h`, `common/dt_validation.c`
- Create: `tests/test_framework.h`
- Create: `tests/CMakeLists.txt`
- Create: `tests/test_dt_validation.c`
- Create: `.gitignore`

**Interfaces:**
- Produces: `timestamp_t { double t_s; }`, `hal_status_t` enum
  (`HAL_OK`, `HAL_NO_NEW_DATA`, `HAL_FAULT`), `#define DT_MAX_S 2.0`,
  `bool dt_is_valid(double dt)`, `double dt_between(timestamp_t a, timestamp_t b)`
  (returns `b.t_s - a.t_s`). All later tasks include `common/types.h` and
  `common/dt_validation.h`.

- [ ] **Step 1: Write `common/types.h`**

```c
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

typedef struct {
    double t_s; /* monotonic seconds, arbitrary epoch */
} timestamp_t;

typedef enum {
    HAL_OK = 0,
    HAL_NO_NEW_DATA,
    HAL_FAULT,
} hal_status_t;

#endif /* COMMON_TYPES_H */
```

- [ ] **Step 2: Write `common/dt_validation.h`**

```c
#ifndef COMMON_DT_VALIDATION_H
#define COMMON_DT_VALIDATION_H

#include <stdbool.h>
#include "common/types.h"

/* Generous outer sanity bound for any computed dt (not a dropout-length
   limiter — see docs/design.md §6.4). */
#define DT_MAX_S 2.0

bool dt_is_valid(double dt);
double dt_between(timestamp_t earlier, timestamp_t later);

#endif /* COMMON_DT_VALIDATION_H */
```

- [ ] **Step 3: Write `common/dt_validation.c`**

```c
#include "common/dt_validation.h"

bool dt_is_valid(double dt) {
    return dt > 0.0 && dt <= DT_MAX_S;
}

double dt_between(timestamp_t earlier, timestamp_t later) {
    return later.t_s - earlier.t_s;
}
```

- [ ] **Step 4: Write `tests/test_framework.h`**

```c
#ifndef TESTS_TEST_FRAMEWORK_H
#define TESTS_TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_current_test_failed = 0;

#define TEST(name) static void name(void)

#define RUN_TEST(name) do { \
    g_current_test_failed = 0; \
    g_tests_run++; \
    name(); \
    if (!g_current_test_failed) { \
        g_tests_passed++; \
    } else { \
        printf("FAIL: %s\n", #name); \
    } \
} while (0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        g_current_test_failed = 1; \
        printf("  CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabs((a) - (b)) <= (tol))

#define TEST_SUMMARY() do { \
    printf("%d/%d tests passing\n", g_tests_passed, g_tests_run); \
    if (g_tests_passed != g_tests_run) return 1; \
} while (0)

#endif /* TESTS_TEST_FRAMEWORK_H */
```

- [ ] **Step 5: Write `tests/test_dt_validation.c`**

```c
#include "tests/test_framework.h"
#include "common/dt_validation.h"

TEST(test_positive_dt_within_bound_is_valid) {
    CHECK(dt_is_valid(0.01));
    CHECK(dt_is_valid(1.99));
}

TEST(test_zero_or_negative_dt_is_invalid) {
    CHECK(!dt_is_valid(0.0));
    CHECK(!dt_is_valid(-0.01));
}

TEST(test_dt_over_max_is_invalid) {
    CHECK(!dt_is_valid(DT_MAX_S + 0.001));
    CHECK(dt_is_valid(DT_MAX_S));
}

TEST(test_dt_between_computes_forward_difference) {
    timestamp_t a = { 1.0 };
    timestamp_t b = { 1.05 };
    CHECK_NEAR(dt_between(a, b), 0.05, 1e-9);
}

int main(void) {
    RUN_TEST(test_positive_dt_within_bound_is_valid);
    RUN_TEST(test_zero_or_negative_dt_is_invalid);
    RUN_TEST(test_dt_over_max_is_invalid);
    RUN_TEST(test_dt_between_computes_forward_difference);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 6: Write top-level `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(tof_imu_docking_rig C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})

enable_testing()
add_subdirectory(tests)
```

- [ ] **Step 7: Write `tests/CMakeLists.txt`**

```cmake
add_library(common_lib STATIC
    ${CMAKE_SOURCE_DIR}/common/dt_validation.c
)

add_executable(test_dt_validation test_dt_validation.c)
target_link_libraries(test_dt_validation common_lib)
add_test(NAME test_dt_validation COMMAND test_dt_validation)
```

- [ ] **Step 8: Write `.gitignore`**

```
build/
*.o
*.csv
```

- [ ] **Step 9: Configure, build, run**

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: `4/4 tests passing`.

- [ ] **Step 10: git init + first commit**

```bash
git init
git add CMakeLists.txt common/ tests/ docs/ CLAUDE.md .gitignore
git commit -m "scaffold: CMake, common types, dt validation, test framework"
```

---

## Task 2: `common/measurement.h` shared measurement types

**Files:**
- Create: `common/measurement.h`

**Interfaces:**
- Consumes: `timestamp_t`, `hal_status_t` (Task 1).
- Produces: `range_sample_t { double range_m; timestamp_t ts; hal_status_t status; }`,
  `imu_sample_t { double accel_mps2[3]; double gyro_rps[3]; timestamp_t ts; hal_status_t status; }`.
  Used by `hal/hal.h` (Task 3) and every `sim/` module (Task 5+).

- [ ] **Step 1: Write `common/measurement.h`**

```c
#ifndef COMMON_MEASUREMENT_H
#define COMMON_MEASUREMENT_H

#include "common/types.h"

typedef struct {
    double range_m;
    timestamp_t ts;
    hal_status_t status;
} range_sample_t;

typedef struct {
    double accel_mps2[3]; /* X=longitudinal, Y=lateral(unused), Z=vertical */
    double gyro_rps[3];   /* [1]=pitch rate, used; [0],[2] unused in V1 */
    timestamp_t ts;
    hal_status_t status;
} imu_sample_t;

#endif /* COMMON_MEASUREMENT_H */
```

- [ ] **Step 2: No test needed (pure data struct, exercised by Task 3+ tests). Commit.**

```bash
git add common/measurement.h
git commit -m "add: shared range/IMU measurement sample types"
```

---

## Task 3: HAL interface contract (`hal/hal.h`) — **CHECKPOINT: confirm with user before continuing**

**Files:**
- Create: `hal/hal.h`

**Interfaces:**
- Consumes: `range_sample_t`, `imu_sample_t` (Task 2), `hal_status_t` (Task 1).
- Produces: `hal_t` struct with function pointers `range_read`, `imu_read`,
  `actuator_set_angle_deg`, `clock_now`, each taking a `void *ctx` first
  arg. This is the *only* type estimation/guidance/logging/main-loop code
  depends on for I/O. `hal_host.c` (Task 6) and any future `hal_esp32.c`
  populate one of these.

- [ ] **Step 1: Write `hal/hal.h`**

```c
#ifndef HAL_HAL_H
#define HAL_HAL_H

#include "common/types.h"
#include "common/measurement.h"

typedef struct {
    void *ctx;

    /* Non-blocking. Returns HAL_OK + fills *out on a fresh sample,
       HAL_NO_NEW_DATA if nothing new since the last call, HAL_FAULT on a
       sensor fault. See docs/design.md §5.1. */
    hal_status_t (*range_read)(void *ctx, range_sample_t *out);

    /* Non-blocking, same status contract. See docs/design.md §5.2. */
    hal_status_t (*imu_read)(void *ctx, imu_sample_t *out);

    /* Non-blocking, fire-and-forget. See docs/design.md §5.3. */
    hal_status_t (*actuator_set_angle_deg)(void *ctx, double angle_deg);

    /* Monotonic seconds, arbitrary epoch. See docs/design.md §5.4. */
    timestamp_t (*clock_now)(void *ctx);
} hal_t;

#endif /* HAL_HAL_H */
```

- [ ] **Step 2: Build to confirm it compiles standalone (no .c to test yet — header-only)**

```bash
cc -std=c99 -c -I. hal/hal.h -o /dev/null -x c 2>&1 | head -20
```

Expected: no errors (a header with only structs/typedefs compiles clean
when included via `-x c`).

- [ ] **Step 3: Commit**

```bash
git add hal/hal.h
git commit -m "add: HAL interface contract (IRangeSensor/IImu/IActuator/IClock)"
```

- [ ] **Step 4: STOP. Present `hal/hal.h` and docs/design.md §5 to the user for confirmation before proceeding to Task 4.** Do not write `sim/` or `hal_host.c` until confirmed.

---

**Task 3 checkpoint: confirmed by user 2026-09-09. Continuing.**

---

## Task 4: `sim/sim_noise.c/h` — deterministic noise generator

**Files:**
- Create: `sim/sim_noise.h`, `sim/sim_noise.c`
- Test: `tests/test_sim_noise.c`

**Interfaces:**
- Produces: `void sim_noise_seed(sim_noise_t *n, uint32_t seed)`,
  `double sim_noise_gaussian(sim_noise_t *n, double mean, double stddev)`.
  Used by every `sim/` module from Task 5 on. A self-contained xorshift32 +
  Box-Muller generator is used (not `rand()`/`<stdlib.h>`) so host tests are
  bit-for-bit reproducible across platforms/libc versions.

- [ ] **Step 1: Write `sim/sim_noise.h`**

```c
#ifndef SIM_SIM_NOISE_H
#define SIM_SIM_NOISE_H

#include <stdint.h>

typedef struct {
    uint32_t state;
} sim_noise_t;

void sim_noise_seed(sim_noise_t *n, uint32_t seed);
double sim_noise_gaussian(sim_noise_t *n, double mean, double stddev);

#endif /* SIM_SIM_NOISE_H */
```

- [ ] **Step 2: Write `sim/sim_noise.c`**

```c
#include "sim/sim_noise.h"
#include <math.h>

static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double uniform01(sim_noise_t *n) {
    return (double)xorshift32(&n->state) / 4294967296.0;
}

void sim_noise_seed(sim_noise_t *n, uint32_t seed) {
    n->state = seed ? seed : 1u; /* xorshift requires nonzero state */
}

double sim_noise_gaussian(sim_noise_t *n, double mean, double stddev) {
    double u1 = uniform01(n);
    double u2 = uniform01(n);
    if (u1 < 1e-12) u1 = 1e-12;
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); /* Box-Muller */
    return mean + stddev * z;
}
```

- [ ] **Step 3: Write `tests/test_sim_noise.c`**

```c
#include "tests/test_framework.h"
#include "sim/sim_noise.h"
#include <math.h>

TEST(test_same_seed_reproduces_same_sequence) {
    sim_noise_t a, b;
    sim_noise_seed(&a, 42);
    sim_noise_seed(&b, 42);
    for (int i = 0; i < 20; i++) {
        CHECK_NEAR(sim_noise_gaussian(&a, 0, 1), sim_noise_gaussian(&b, 0, 1), 1e-12);
    }
}

TEST(test_zero_stddev_returns_mean_exactly) {
    sim_noise_t n;
    sim_noise_seed(&n, 7);
    for (int i = 0; i < 10; i++) {
        CHECK_NEAR(sim_noise_gaussian(&n, 3.5, 0.0), 3.5, 1e-12);
    }
}

TEST(test_sample_mean_converges_toward_requested_mean) {
    sim_noise_t n;
    sim_noise_seed(&n, 123);
    double sum = 0.0;
    int count = 20000;
    for (int i = 0; i < count; i++) sum += sim_noise_gaussian(&n, 2.0, 0.5);
    CHECK_NEAR(sum / count, 2.0, 0.05);
}

int main(void) {
    RUN_TEST(test_same_seed_reproduces_same_sequence);
    RUN_TEST(test_zero_stddev_returns_mean_exactly);
    RUN_TEST(test_sample_mean_converges_toward_requested_mean);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(sim_noise_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_noise.c)
target_link_libraries(sim_noise_lib m)

add_executable(test_sim_noise test_sim_noise.c)
target_link_libraries(test_sim_noise sim_noise_lib m)
add_test(NAME test_sim_noise COMMAND test_sim_noise)
```

- [ ] **Step 5: Build and run; expect `4/4` new + prior `4/4` = report both suites' totals ("test_dt_validation: 4/4", "test_sim_noise: 3/3")**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add sim/sim_noise.h sim/sim_noise.c tests/test_sim_noise.c tests/CMakeLists.txt
git commit -m "add: deterministic sim noise generator (xorshift32 + Box-Muller)"
```

---

## Task 5: `sim/sim_cart.c/h` — ground-truth cart dynamics

**Files:**
- Create: `sim/sim_cart.h`, `sim/sim_cart.c`
- Test: `tests/test_sim_cart.c`

**Interfaces:**
- Consumes: nothing beyond `<stdbool.h>`.
- Produces: `sim_cart_t { double true_range_m; double true_velocity_mps; double track_length_m; }`,
  `void sim_cart_init(sim_cart_t *c, double initial_range_m, double track_length_m)`,
  `void sim_cart_step(sim_cart_t *c, double true_accel_mps2, double dt)`.
  Consumed by `hal/hal_host.c` (Task 8) as the ground-truth source, and by
  the estimation/control measurement-contract tests (Tasks 12, 16) — those
  tests read `true_range_m`/`true_velocity_mps` directly, bypassing the
  HAL, per docs/design.md §8.

- [ ] **Step 1: Write `sim/sim_cart.h`**

```c
#ifndef SIM_SIM_CART_H
#define SIM_SIM_CART_H

typedef struct {
    double true_range_m;     /* distance to target, >= 0, decreases approaching */
    double true_velocity_mps; /* positive toward target */
    double track_length_m;
} sim_cart_t;

void sim_cart_init(sim_cart_t *c, double initial_range_m, double track_length_m);

/* true_accel_mps2: positive toward target. Integrates velocity then range.
   Clamps range to [0, track_length_m]. */
void sim_cart_step(sim_cart_t *c, double true_accel_mps2, double dt);

#endif /* SIM_SIM_CART_H */
```

- [ ] **Step 2: Write `sim/sim_cart.c`**

```c
#include "sim/sim_cart.h"

void sim_cart_init(sim_cart_t *c, double initial_range_m, double track_length_m) {
    c->true_range_m = initial_range_m;
    c->true_velocity_mps = 0.0;
    c->track_length_m = track_length_m;
}

void sim_cart_step(sim_cart_t *c, double true_accel_mps2, double dt) {
    c->true_velocity_mps += true_accel_mps2 * dt;
    c->true_range_m -= c->true_velocity_mps * dt; /* +velocity toward target shrinks range */
    if (c->true_range_m < 0.0) c->true_range_m = 0.0;
    if (c->true_range_m > c->track_length_m) c->true_range_m = c->track_length_m;
}
```

- [ ] **Step 3: Write `tests/test_sim_cart.c`**

```c
#include "tests/test_framework.h"
#include "sim/sim_cart.h"

TEST(test_init_sets_initial_state) {
    sim_cart_t c;
    sim_cart_init(&c, 1.0, 1.0);
    CHECK_NEAR(c.true_range_m, 1.0, 1e-12);
    CHECK_NEAR(c.true_velocity_mps, 0.0, 1e-12);
}

TEST(test_positive_accel_increases_velocity_and_decreases_range) {
    sim_cart_t c;
    sim_cart_init(&c, 1.0, 1.0);
    sim_cart_step(&c, 0.5, 0.1); /* v += 0.05 -> 0.05; r -= 0.05*0.1 = 0.005 */
    CHECK_NEAR(c.true_velocity_mps, 0.05, 1e-9);
    CHECK_NEAR(c.true_range_m, 1.0 - 0.005, 1e-9);
}

TEST(test_range_clamped_at_zero_on_overshoot) {
    sim_cart_t c;
    sim_cart_init(&c, 0.01, 1.0);
    sim_cart_step(&c, 0.0, 0.01); /* velocity 0, no motion yet; force velocity via prior accel */
    sim_cart_step(&c, 100.0, 0.1); /* large accel -> large velocity -> range would go negative */
    CHECK(c.true_range_m >= 0.0);
}

TEST(test_range_clamped_at_track_length) {
    sim_cart_t c;
    sim_cart_init(&c, 0.0, 1.0);
    sim_cart_step(&c, -100.0, 0.1); /* negative accel -> negative velocity -> range grows past track */
    CHECK(c.true_range_m <= 1.0);
}

int main(void) {
    RUN_TEST(test_init_sets_initial_state);
    RUN_TEST(test_positive_accel_increases_velocity_and_decreases_range);
    RUN_TEST(test_range_clamped_at_zero_on_overshoot);
    RUN_TEST(test_range_clamped_at_track_length);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(sim_cart_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_cart.c)

add_executable(test_sim_cart test_sim_cart.c)
target_link_libraries(test_sim_cart sim_cart_lib)
add_test(NAME test_sim_cart COMMAND test_sim_cart)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add sim/sim_cart.h sim/sim_cart.c tests/test_sim_cart.c tests/CMakeLists.txt
git commit -m "add: ground-truth cart dynamics sim"
```

---

## Task 6: `sim/sim_range_sensor.c/h` — simulated ToF sensor

**Files:**
- Create: `sim/sim_range_sensor.h`, `sim/sim_range_sensor.c`
- Test: `tests/test_sim_range_sensor.c`

**Interfaces:**
- Consumes: `sim_noise_t`/`sim_noise_gaussian` (Task 4), `timestamp_t`,
  `range_sample_t`, `hal_status_t` (Tasks 1–2).
- Produces: `sim_range_sensor_t`, `sim_range_sensor_init(...)`,
  `hal_status_t sim_range_sensor_sample(sim_range_sensor_t *s, double true_range_m, timestamp_t now, range_sample_t *out)`,
  `void sim_range_sensor_set_dropout(sim_range_sensor_t *s, bool active)`.
  Consumed by `hal/hal_host.c` (Task 8).

- [ ] **Step 1: Write `sim/sim_range_sensor.h`**

```c
#ifndef SIM_SIM_RANGE_SENSOR_H
#define SIM_SIM_RANGE_SENSOR_H

#include <stdbool.h>
#include "common/types.h"
#include "common/measurement.h"
#include "sim/sim_noise.h"

typedef struct {
    double period_s;         /* nominal sensor conversion period, e.g. 0.05 (20Hz) */
    double noise_stddev_m;
    double last_emit_t_s;    /* -1 before first sample */
    bool dropout_active;     /* test hook: force HAL_NO_NEW_DATA regardless of period */
    sim_noise_t noise;
} sim_range_sensor_t;

void sim_range_sensor_init(sim_range_sensor_t *s, double period_s, double noise_stddev_m, uint32_t seed);

/* Emits HAL_OK with a fresh noisy sample once per period_s of sim time
   elapsed since the last emission; HAL_NO_NEW_DATA otherwise or while a
   dropout is active. */
hal_status_t sim_range_sensor_sample(sim_range_sensor_t *s, double true_range_m, timestamp_t now, range_sample_t *out);

void sim_range_sensor_set_dropout(sim_range_sensor_t *s, bool active);

#endif /* SIM_SIM_RANGE_SENSOR_H */
```

- [ ] **Step 2: Write `sim/sim_range_sensor.c`**

```c
#include "sim/sim_range_sensor.h"

void sim_range_sensor_init(sim_range_sensor_t *s, double period_s, double noise_stddev_m, uint32_t seed) {
    s->period_s = period_s;
    s->noise_stddev_m = noise_stddev_m;
    s->last_emit_t_s = -1.0;
    s->dropout_active = false;
    sim_noise_seed(&s->noise, seed);
}

void sim_range_sensor_set_dropout(sim_range_sensor_t *s, bool active) {
    s->dropout_active = active;
}

hal_status_t sim_range_sensor_sample(sim_range_sensor_t *s, double true_range_m, timestamp_t now, range_sample_t *out) {
    if (s->dropout_active) {
        return HAL_NO_NEW_DATA;
    }
    bool due = (s->last_emit_t_s < 0.0) || (now.t_s - s->last_emit_t_s >= s->period_s);
    if (!due) {
        return HAL_NO_NEW_DATA;
    }
    s->last_emit_t_s = now.t_s;
    out->range_m = true_range_m + sim_noise_gaussian(&s->noise, 0.0, s->noise_stddev_m);
    if (out->range_m < 0.0) out->range_m = 0.0;
    out->ts = now;
    out->status = HAL_OK;
    return HAL_OK;
}
```

- [ ] **Step 3: Write `tests/test_sim_range_sensor.c`**

```c
#include "tests/test_framework.h"
#include "sim/sim_range_sensor.h"

TEST(test_no_sample_before_first_period_elapses) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    CHECK(sim_range_sensor_sample(&s, 1.0, t0, &out) == HAL_OK); /* first call always due */
    timestamp_t t1 = { 0.01 };
    CHECK(sim_range_sensor_sample(&s, 1.0, t1, &out) == HAL_NO_NEW_DATA);
}

TEST(test_sample_emitted_once_period_elapses) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 1.0, t0, &out);
    timestamp_t t1 = { 0.05 };
    CHECK(sim_range_sensor_sample(&s, 0.9, t1, &out) == HAL_OK);
    CHECK_NEAR(out.range_m, 0.9, 1e-9);
    CHECK_NEAR(out.ts.t_s, 0.05, 1e-9);
}

TEST(test_dropout_forces_no_new_data_even_when_due) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 1.0, t0, &out);
    sim_range_sensor_set_dropout(&s, true);
    timestamp_t t1 = { 0.5 };
    CHECK(sim_range_sensor_sample(&s, 0.5, t1, &out) == HAL_NO_NEW_DATA);
    sim_range_sensor_set_dropout(&s, false);
    timestamp_t t2 = { 0.55 };
    CHECK(sim_range_sensor_sample(&s, 0.5, t2, &out) == HAL_OK);
}

TEST(test_noise_free_sample_matches_true_range_exactly) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 0.75, t0, &out);
    CHECK_NEAR(out.range_m, 0.75, 1e-12);
}

int main(void) {
    RUN_TEST(test_no_sample_before_first_period_elapses);
    RUN_TEST(test_sample_emitted_once_period_elapses);
    RUN_TEST(test_dropout_forces_no_new_data_even_when_due);
    RUN_TEST(test_noise_free_sample_matches_true_range_exactly);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(sim_range_sensor_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_range_sensor.c)
target_link_libraries(sim_range_sensor_lib sim_noise_lib m)

add_executable(test_sim_range_sensor test_sim_range_sensor.c)
target_link_libraries(test_sim_range_sensor sim_range_sensor_lib m)
add_test(NAME test_sim_range_sensor COMMAND test_sim_range_sensor)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add sim/sim_range_sensor.h sim/sim_range_sensor.c tests/test_sim_range_sensor.c tests/CMakeLists.txt
git commit -m "add: simulated ToF range sensor (rate-limited, noisy, dropout hook)"
```

---

## Task 7: `sim/sim_imu.c/h` — simulated IMU (gravity model, noise, bias)

**Files:**
- Create: `sim/sim_imu.h`, `sim/sim_imu.c`
- Test: `tests/test_sim_imu.c`

**Interfaces:**
- Consumes: `sim_noise_t` (Task 4), `imu_sample_t`, `timestamp_t`,
  `hal_status_t` (Tasks 1–2).
- Produces: `sim_imu_t`, `sim_imu_init(...)`,
  `hal_status_t sim_imu_sample(sim_imu_t *s, double true_accel_long_mps2, double true_theta_rad, double true_omega_y_rps, timestamp_t now, imu_sample_t *out)`,
  `void sim_imu_set_accel_bias(sim_imu_t *s, double bias_mps2)` (test hook
  for the IMU bias-drift scenario, Task 12). Consumed by `hal/hal_host.c`
  (Task 8). The forward model here is the algebraic inverse of
  `estimation/gravity_compensation.c` (Task 9) — see derivation comment in
  Step 2 — so a correct gravity-compensation implementation exactly
  recovers `true_accel_long_mps2` from noise-free, bias-free output.

- [ ] **Step 1: Write `sim/sim_imu.h`**

```c
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
```

- [ ] **Step 2: Write `sim/sim_imu.c`**

```c
#include "sim/sim_imu.h"
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
```

- [ ] **Step 3: Write `tests/test_sim_imu.c`**

```c
#include "tests/test_framework.h"
#include "sim/sim_imu.h"
#include <math.h>

TEST(test_level_noise_free_accel_x_equals_true_accel) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.3, 0.0 /* theta */, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], 0.3, 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2, 1e-9);
}

TEST(test_pitched_accel_matches_derived_rotation_model) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    double theta = 0.1;
    sim_imu_sample(&s, 0.0, theta, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], SIM_GRAVITY_MPS2 * sin(theta), 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2 * cos(theta), 1e-9);
}

TEST(test_gyro_y_matches_true_omega) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.25, t0, &out);
    CHECK_NEAR(out.gyro_rps[1], 0.25, 1e-9);
}

TEST(test_no_sample_before_period_elapses) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.0, t0, &out);
    timestamp_t t1 = { 0.001 };
    CHECK(sim_imu_sample(&s, 0.0, 0.0, 0.0, t1, &out) == HAL_NO_NEW_DATA);
}

TEST(test_accel_bias_offsets_x_axis_only) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    sim_imu_set_accel_bias(&s, 0.2);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], 0.2, 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2, 1e-9);
}

int main(void) {
    RUN_TEST(test_level_noise_free_accel_x_equals_true_accel);
    RUN_TEST(test_pitched_accel_matches_derived_rotation_model);
    RUN_TEST(test_gyro_y_matches_true_omega);
    RUN_TEST(test_no_sample_before_period_elapses);
    RUN_TEST(test_accel_bias_offsets_x_axis_only);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(sim_imu_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_imu.c)
target_link_libraries(sim_imu_lib sim_noise_lib m)

add_executable(test_sim_imu test_sim_imu.c)
target_link_libraries(test_sim_imu sim_imu_lib m)
add_test(NAME test_sim_imu COMMAND test_sim_imu)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add sim/sim_imu.h sim/sim_imu.c tests/test_sim_imu.c tests/CMakeLists.txt
git commit -m "add: simulated IMU with gravity-coupled pitch model, noise, bias hook"
```

---

## Task 8: `hal/hal_host.c/h` — host HAL backend (wires sim/ into hal_t)

**Files:**
- Create: `hal/hal_host.h`, `hal/hal_host.c`
- Test: `tests/test_hal_host.c`

**Interfaces:**
- Consumes: `hal_t` (Task 3), `sim_cart_t`/`sim_cart_step` (Task 5),
  `sim_range_sensor_t`/`sim_range_sensor_sample` (Task 6),
  `sim_imu_t`/`sim_imu_sample` (Task 7).
- Produces: `hal_host_world_t` (owns all sim state + last commanded servo
  angle + ground-truth pitch state), `void hal_host_world_init(hal_host_world_t *w, double initial_range_m, double track_length_m, uint32_t seed)`,
  `hal_t hal_host_create(hal_host_world_t *w)`,
  `void hal_host_world_tick(hal_host_world_t *w, double true_accel_mps2, double true_omega_y_rps, double dt)`
  (test/main-loop driver: advances clock, steps `sim_cart`, integrates
  ground-truth pitch, stashes `true_accel_mps2` for this tick's IMU read).
  `main_host_sim.c` (Task 18) and the chain-metrics tests (Tasks 12, 16)
  call `hal_host_world_tick` once per main-loop iteration, then read
  through the returned `hal_t`.

- [ ] **Step 1: Write `hal/hal_host.h`**

```c
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
```

- [ ] **Step 2: Write `hal/hal_host.c`**

```c
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

hal_t hal_host_create(hal_host_world_t *w) {
    hal_t h;
    h.ctx = w;
    h.range_read = host_range_read;
    h.imu_read = host_imu_read;
    h.actuator_set_angle_deg = host_actuator_set_angle_deg;
    h.clock_now = host_clock_now;
    return h;
}
```

- [ ] **Step 3: Write `tests/test_hal_host.c`**

```c
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

int main(void) {
    RUN_TEST(test_hal_t_reads_route_through_world_state);
    RUN_TEST(test_actuator_command_is_captured);
    RUN_TEST(test_clock_now_reflects_ticks);
    RUN_TEST(test_tick_advances_true_cart_state);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(hal_host_lib STATIC ${CMAKE_SOURCE_DIR}/hal/hal_host.c)
target_link_libraries(hal_host_lib sim_cart_lib sim_range_sensor_lib sim_imu_lib m)

add_executable(test_hal_host test_hal_host.c)
target_link_libraries(test_hal_host hal_host_lib m)
add_test(NAME test_hal_host COMMAND test_hal_host)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add hal/hal_host.h hal/hal_host.c tests/test_hal_host.c tests/CMakeLists.txt
git commit -m "add: host HAL backend wiring sim/ into hal_t"
```

---

## Task 9: `estimation/gravity_compensation.c/h`

**Files:**
- Create: `estimation/gravity_compensation.h`, `estimation/gravity_compensation.c`
- Test: `tests/test_gravity_compensation.c`

**Interfaces:**
- Consumes: nothing beyond `<math.h>`.
- Produces: `double gravity_compensate(double a_x_mps2, double a_z_mps2, double theta_rad)`.
  Consumed by `estimation/complementary_filter.c` (Task 12).

- [ ] **Step 1: Write `estimation/gravity_compensation.h`**

```c
#ifndef ESTIMATION_GRAVITY_COMPENSATION_H
#define ESTIMATION_GRAVITY_COMPENSATION_H

/* a_longitudinal = a_x*cos(theta) - a_z*sin(theta), using the 1D pitch
   theta (docs/design.md §3.1, §6.2). Own named module per working
   agreement — not inlined into the fusion code. */
double gravity_compensate(double a_x_mps2, double a_z_mps2, double theta_rad);

#endif /* ESTIMATION_GRAVITY_COMPENSATION_H */
```

- [ ] **Step 2: Write `estimation/gravity_compensation.c`**

```c
#include "estimation/gravity_compensation.h"
#include <math.h>

double gravity_compensate(double a_x_mps2, double a_z_mps2, double theta_rad) {
    return a_x_mps2 * cos(theta_rad) - a_z_mps2 * sin(theta_rad);
}
```

- [ ] **Step 3: Write `tests/test_gravity_compensation.c`** (values cross-checked against `sim/sim_imu.c`'s forward model, Task 7)

```c
#include "tests/test_framework.h"
#include "estimation/gravity_compensation.h"
#include "sim/sim_imu.h"
#include <math.h>

TEST(test_level_case_passes_ax_through_unchanged) {
    CHECK_NEAR(gravity_compensate(0.4, SIM_GRAVITY_MPS2, 0.0), 0.4, 1e-9);
}

TEST(test_recovers_true_accel_through_sim_imu_forward_model_at_various_pitches) {
    double thetas[] = { -0.3, -0.1, 0.0, 0.1, 0.3 };
    double true_accels[] = { -0.5, 0.0, 0.2, 0.6 };
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            double theta = thetas[i];
            double true_a = true_accels[j];
            double a_x =  true_a * cos(theta) + SIM_GRAVITY_MPS2 * sin(theta);
            double a_z = -true_a * sin(theta) + SIM_GRAVITY_MPS2 * cos(theta);
            CHECK_NEAR(gravity_compensate(a_x, a_z, theta), true_a, 1e-9);
        }
    }
}

int main(void) {
    RUN_TEST(test_level_case_passes_ax_through_unchanged);
    RUN_TEST(test_recovers_true_accel_through_sim_imu_forward_model_at_various_pitches);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(gravity_compensation_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/gravity_compensation.c)
target_link_libraries(gravity_compensation_lib m)

add_executable(test_gravity_compensation test_gravity_compensation.c)
target_link_libraries(test_gravity_compensation gravity_compensation_lib sim_imu_lib m)
add_test(NAME test_gravity_compensation COMMAND test_gravity_compensation)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add estimation/gravity_compensation.h estimation/gravity_compensation.c tests/test_gravity_compensation.c tests/CMakeLists.txt
git commit -m "add: gravity compensation module"
```

---

## Task 10: `estimation/orientation_1d.c/h` — 1D pitch integration

**Files:**
- Create: `estimation/orientation_1d.h`, `estimation/orientation_1d.c`
- Test: `tests/test_orientation_1d.c`

**Interfaces:**
- Consumes: `dt_is_valid` (Task 1).
- Produces: `typedef struct { double theta_rad; } orientation_1d_t;`,
  `void orientation_1d_init(orientation_1d_t *o)`,
  `void orientation_1d_update(orientation_1d_t *o, double omega_y_rps, double dt)`
  (no-op — holds `theta_rad` — on invalid dt, per docs/design.md §6.4).
  Consumed by `estimation/complementary_filter.c` (Task 12).

- [ ] **Step 1: Write `estimation/orientation_1d.h`**

```c
#ifndef ESTIMATION_ORIENTATION_1D_H
#define ESTIMATION_ORIENTATION_1D_H

/* Cart is rail-constrained: no roll/yaw DOF exists to estimate, so
   orientation is a single scalar pitch about Y, not a 3D attitude
   (docs/design.md §3.1). theta feeds gravity_compensation.c only. */
typedef struct {
    double theta_rad;
} orientation_1d_t;

void orientation_1d_init(orientation_1d_t *o);

/* theta_k = theta_{k-1} + omega_y*dt. On invalid dt (docs/design.md §6.4),
   holds theta_rad unchanged rather than integrating over a bad dt. */
void orientation_1d_update(orientation_1d_t *o, double omega_y_rps, double dt);

#endif /* ESTIMATION_ORIENTATION_1D_H */
```

- [ ] **Step 2: Write `estimation/orientation_1d.c`**

```c
#include "estimation/orientation_1d.h"
#include "common/dt_validation.h"

void orientation_1d_init(orientation_1d_t *o) {
    o->theta_rad = 0.0;
}

void orientation_1d_update(orientation_1d_t *o, double omega_y_rps, double dt) {
    if (!dt_is_valid(dt)) {
        return; /* hold last state */
    }
    o->theta_rad += omega_y_rps * dt;
}
```

- [ ] **Step 3: Write `tests/test_orientation_1d.c`**

```c
#include "tests/test_framework.h"
#include "estimation/orientation_1d.h"

TEST(test_init_starts_at_zero) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    CHECK_NEAR(o.theta_rad, 0.0, 1e-12);
}

TEST(test_update_integrates_omega_over_dt) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    orientation_1d_update(&o, 0.5, 0.1);
    CHECK_NEAR(o.theta_rad, 0.05, 1e-9);
    orientation_1d_update(&o, 0.5, 0.1);
    CHECK_NEAR(o.theta_rad, 0.10, 1e-9);
}

TEST(test_invalid_dt_holds_theta) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    orientation_1d_update(&o, 0.5, 0.1);
    double before = o.theta_rad;
    orientation_1d_update(&o, 0.5, 0.0);   /* dt<=0 */
    orientation_1d_update(&o, 0.5, 3.0);   /* dt>DT_MAX_S */
    CHECK_NEAR(o.theta_rad, before, 1e-12);
}

int main(void) {
    RUN_TEST(test_init_starts_at_zero);
    RUN_TEST(test_update_integrates_omega_over_dt);
    RUN_TEST(test_invalid_dt_holds_theta);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(orientation_1d_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/orientation_1d.c)
target_link_libraries(orientation_1d_lib common_lib)

add_executable(test_orientation_1d test_orientation_1d.c)
target_link_libraries(test_orientation_1d orientation_1d_lib)
add_test(NAME test_orientation_1d COMMAND test_orientation_1d)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add estimation/orientation_1d.h estimation/orientation_1d.c tests/test_orientation_1d.c tests/CMakeLists.txt
git commit -m "add: 1D pitch orientation integration with dt validation"
```

---

## Task 11: `estimation/raw_speed.c/h` — backward-difference RAW speed + staleness

**Files:**
- Create: `estimation/raw_speed.h`, `estimation/raw_speed.c`
- Test: `tests/test_raw_speed.c`

**Interfaces:**
- Consumes: `dt_is_valid`, `dt_between` (Task 1), `timestamp_t` (Task 1).
- Produces: `typedef struct { double value_mps; bool is_stale; } raw_speed_t;`,
  `typedef struct { double last_range_m; timestamp_t last_ts; bool has_prev; raw_speed_t speed; } raw_speed_estimator_t;`,
  `void raw_speed_init(raw_speed_estimator_t *e)`,
  `void raw_speed_on_new_range(raw_speed_estimator_t *e, double range_m, timestamp_t ts)`
  (call only when a fresh ToF sample arrived; recomputes and clears
  `is_stale`) , `void raw_speed_mark_stale(raw_speed_estimator_t *e)` (call
  on ticks with no new ToF sample; holds `value_mps`, sets `is_stale`).
  Consumed by `estimation/complementary_filter.c` (Task 12) and
  `logging/csv_logger.c` (Task 17).

- [ ] **Step 1: Write `estimation/raw_speed.h`**

```c
#ifndef ESTIMATION_RAW_SPEED_H
#define ESTIMATION_RAW_SPEED_H

#include <stdbool.h>
#include "common/types.h"

typedef struct {
    double value_mps;
    bool is_stale; /* true = held from a previous tick, not recomputed this tick */
} raw_speed_t;

typedef struct {
    double last_range_m;
    timestamp_t last_ts;
    bool has_prev;
    raw_speed_t speed;
} raw_speed_estimator_t;

void raw_speed_init(raw_speed_estimator_t *e);

/* Call ONLY on ticks where a genuinely new ToF sample arrived
   (docs/design.md §6.1). Computes (r_k - r_{k-1})/(t_k - t_{k-1}) subject
   to dt validation; on an invalid dt or no previous sample yet, holds/
   flags stale instead of dividing by a bad or nonexistent dt. */
void raw_speed_on_new_range(raw_speed_estimator_t *e, double range_m, timestamp_t ts);

/* Call on every tick with no new ToF sample: holds value_mps, sets is_stale. */
void raw_speed_mark_stale(raw_speed_estimator_t *e);

#endif /* ESTIMATION_RAW_SPEED_H */
```

- [ ] **Step 2: Write `estimation/raw_speed.c`**

```c
#include "estimation/raw_speed.h"
#include "common/dt_validation.h"

void raw_speed_init(raw_speed_estimator_t *e) {
    e->last_range_m = 0.0;
    e->last_ts.t_s = 0.0;
    e->has_prev = false;
    e->speed.value_mps = 0.0;
    e->speed.is_stale = true; /* nothing computed yet */
}

void raw_speed_on_new_range(raw_speed_estimator_t *e, double range_m, timestamp_t ts) {
    if (!e->has_prev) {
        e->last_range_m = range_m;
        e->last_ts = ts;
        e->has_prev = true;
        e->speed.is_stale = true; /* no prior sample to difference against */
        return;
    }
    double dt = dt_between(e->last_ts, ts);
    if (dt_is_valid(dt)) {
        e->speed.value_mps = (range_m - e->last_range_m) / dt;
        e->speed.is_stale = false;
    } else {
        e->speed.is_stale = true; /* hold previous value_mps, do not divide by bad dt */
    }
    e->last_range_m = range_m;
    e->last_ts = ts;
}

void raw_speed_mark_stale(raw_speed_estimator_t *e) {
    e->speed.is_stale = true; /* value_mps left unchanged (held) */
}
```

- [ ] **Step 3: Write `tests/test_raw_speed.c`**

```c
#include "tests/test_framework.h"
#include "estimation/raw_speed.h"

TEST(test_first_sample_is_stale_no_prior_to_difference) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    raw_speed_on_new_range(&e, 1.0, t0);
    CHECK(e.speed.is_stale);
}

TEST(test_second_sample_computes_backward_difference) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    timestamp_t t1 = { 0.05 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.95, t1); /* range decreased 0.05 over 0.05s -> +1.0 m/s */
    CHECK(!e.speed.is_stale);
    CHECK_NEAR(e.speed.value_mps, 1.0, 1e-9);
}

TEST(test_mark_stale_holds_last_value) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    timestamp_t t1 = { 0.05 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.95, t1);
    double held = e.speed.value_mps;
    raw_speed_mark_stale(&e);
    CHECK(e.speed.is_stale);
    CHECK_NEAR(e.speed.value_mps, held, 1e-12);
}

TEST(test_invalid_dt_holds_and_flags_stale_instead_of_dividing) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.9, t0); /* duplicate timestamp: dt=0 */
    CHECK(e.speed.is_stale);
}

int main(void) {
    RUN_TEST(test_first_sample_is_stale_no_prior_to_difference);
    RUN_TEST(test_second_sample_computes_backward_difference);
    RUN_TEST(test_mark_stale_holds_last_value);
    RUN_TEST(test_invalid_dt_holds_and_flags_stale_instead_of_dividing);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(raw_speed_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/raw_speed.c)
target_link_libraries(raw_speed_lib common_lib)

add_executable(test_raw_speed test_raw_speed.c)
target_link_libraries(test_raw_speed raw_speed_lib)
add_test(NAME test_raw_speed COMMAND test_raw_speed)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add estimation/raw_speed.h estimation/raw_speed.c tests/test_raw_speed.c tests/CMakeLists.txt
git commit -m "add: RAW speed backward differencing with staleness handling"
```

---

## Task 12: `common/metrics.c/h` — shared RMSE/bias/variance/jitter/settling-time helpers

**Files:**
- Create: `common/metrics.h`, `common/metrics.c`
- Test: `tests/test_metrics.c`

**Interfaces:**
- Consumes: nothing beyond `<math.h>`/`<stdbool.h>`.
- Produces: `metrics_accum_t` (+ `metrics_init`, `metrics_add`,
  `metrics_rmse`, `metrics_mean_bias`, `metrics_variance` — `metrics_add`
  with `truth=0` also gives raw-signal mean/variance, reused for
  `control_output` variance in Task 19), `jitter_accum_t` (+
  `jitter_init`, `jitter_add`, `jitter_rms` — RMS of consecutive
  differences), and `double find_settling_time(const double *t_s, const double *values, int n, double target, double tolerance)`
  (first `t_s[i]` after which `values` stays within `tolerance` of `target`
  for the rest of the array; `-1.0` if it never settles). Consumed by
  Tasks 15 and 19 (estimation- and control-chain measurement-contract
  tests) — this is exactly the §8 evaluation contract, implemented once.

- [ ] **Step 1: Write `common/metrics.h`**

```c
#ifndef COMMON_METRICS_H
#define COMMON_METRICS_H

typedef struct {
    double sum_error;
    double sum_sq_error;
    int n;
} metrics_accum_t;

void metrics_init(metrics_accum_t *m);
/* error = estimate - truth. Pass truth=0 to accumulate raw-signal stats
   (mean_bias becomes the signal's mean, variance becomes its variance). */
void metrics_add(metrics_accum_t *m, double estimate, double truth);
double metrics_rmse(const metrics_accum_t *m);
double metrics_mean_bias(const metrics_accum_t *m);
double metrics_variance(const metrics_accum_t *m); /* RMSE^2 = bias^2 + variance */

typedef struct {
    double last_value;
    bool has_prev;
    double sum_sq_diff;
    int n;
} jitter_accum_t;

void jitter_init(jitter_accum_t *j);
void jitter_add(jitter_accum_t *j, double value);
double jitter_rms(const jitter_accum_t *j); /* RMS of consecutive-sample differences */

/* First t_s[i] after which values[] stays within tolerance of target for
   every remaining sample; -1.0 if it never does. */
double find_settling_time(const double *t_s, const double *values, int n, double target, double tolerance);

#endif /* COMMON_METRICS_H */
```

- [ ] **Step 2: Write `common/metrics.c`**

```c
#include "common/metrics.h"
#include <math.h>
#include <stdbool.h>

void metrics_init(metrics_accum_t *m) {
    m->sum_error = 0.0;
    m->sum_sq_error = 0.0;
    m->n = 0;
}

void metrics_add(metrics_accum_t *m, double estimate, double truth) {
    double error = estimate - truth;
    m->sum_error += error;
    m->sum_sq_error += error * error;
    m->n += 1;
}

double metrics_rmse(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    return sqrt(m->sum_sq_error / m->n);
}

double metrics_mean_bias(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    return m->sum_error / m->n;
}

double metrics_variance(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    double bias = metrics_mean_bias(m);
    double mean_sq = m->sum_sq_error / m->n;
    return mean_sq - bias * bias;
}

void jitter_init(jitter_accum_t *j) {
    j->last_value = 0.0;
    j->has_prev = false;
    j->sum_sq_diff = 0.0;
    j->n = 0;
}

void jitter_add(jitter_accum_t *j, double value) {
    if (j->has_prev) {
        double diff = value - j->last_value;
        j->sum_sq_diff += diff * diff;
        j->n += 1;
    }
    j->last_value = value;
    j->has_prev = true;
}

double jitter_rms(const jitter_accum_t *j) {
    if (j->n == 0) return 0.0;
    return sqrt(j->sum_sq_diff / j->n);
}

double find_settling_time(const double *t_s, const double *values, int n, double target, double tolerance) {
    for (int i = 0; i < n; i++) {
        bool stays_within = true;
        for (int k = i; k < n; k++) {
            if (fabs(values[k] - target) > tolerance) {
                stays_within = false;
                break;
            }
        }
        if (stays_within) return t_s[i];
    }
    return -1.0;
}
```

- [ ] **Step 3: Write `tests/test_metrics.c`**

```c
#include "tests/test_framework.h"
#include "common/metrics.h"

TEST(test_rmse_bias_variance_on_known_errors) {
    metrics_accum_t m;
    metrics_init(&m);
    metrics_add(&m, 1.0, 0.0); /* error 1 */
    metrics_add(&m, 3.0, 0.0); /* error 3 */
    /* errors {1,3}: mean=2, mean_sq=(1+9)/2=5, rmse=sqrt(5), variance=5-4=1 */
    CHECK_NEAR(metrics_mean_bias(&m), 2.0, 1e-9);
    CHECK_NEAR(metrics_rmse(&m), 2.2360679775, 1e-6);
    CHECK_NEAR(metrics_variance(&m), 1.0, 1e-9);
}

TEST(test_rmse_squared_equals_bias_squared_plus_variance) {
    metrics_accum_t m;
    metrics_init(&m);
    double estimates[] = { 0.9, 1.2, 1.0, 0.8, 1.3 };
    double truth = 1.0;
    for (int i = 0; i < 5; i++) metrics_add(&m, estimates[i], truth);
    double rmse = metrics_rmse(&m);
    double bias = metrics_mean_bias(&m);
    double var = metrics_variance(&m);
    CHECK_NEAR(rmse * rmse, bias * bias + var, 1e-9);
}

TEST(test_jitter_rms_zero_for_constant_signal) {
    jitter_accum_t j;
    jitter_init(&j);
    for (int i = 0; i < 5; i++) jitter_add(&j, 0.5);
    CHECK_NEAR(jitter_rms(&j), 0.0, 1e-12);
}

TEST(test_jitter_rms_nonzero_for_alternating_signal) {
    jitter_accum_t j;
    jitter_init(&j);
    double vals[] = { 0.0, 1.0, 0.0, 1.0, 0.0 };
    for (int i = 0; i < 5; i++) jitter_add(&j, vals[i]);
    CHECK_NEAR(jitter_rms(&j), 1.0, 1e-9);
}

TEST(test_settling_time_finds_first_stable_crossing) {
    double t[]  = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 };
    double v[]  = { 5.0, 3.0, 1.05, 0.98, 1.01, 1.0 };
    double settle = find_settling_time(t, v, 6, 1.0, 0.1);
    CHECK_NEAR(settle, 0.2, 1e-9);
}

TEST(test_settling_time_returns_negative_one_if_never_settles) {
    double t[] = { 0.0, 0.1, 0.2 };
    double v[] = { 5.0, 5.0, 5.0 };
    CHECK_NEAR(find_settling_time(t, v, 3, 1.0, 0.1), -1.0, 1e-12);
}

int main(void) {
    RUN_TEST(test_rmse_bias_variance_on_known_errors);
    RUN_TEST(test_rmse_squared_equals_bias_squared_plus_variance);
    RUN_TEST(test_jitter_rms_zero_for_constant_signal);
    RUN_TEST(test_jitter_rms_nonzero_for_alternating_signal);
    RUN_TEST(test_settling_time_finds_first_stable_crossing);
    RUN_TEST(test_settling_time_returns_negative_one_if_never_settles);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(metrics_lib STATIC ${CMAKE_SOURCE_DIR}/common/metrics.c)
target_link_libraries(metrics_lib m)

add_executable(test_metrics test_metrics.c)
target_link_libraries(test_metrics metrics_lib m)
add_test(NAME test_metrics COMMAND test_metrics)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add common/metrics.h common/metrics.c tests/test_metrics.c tests/CMakeLists.txt
git commit -m "add: shared RMSE/bias/variance/jitter/settling-time metrics helpers"
```

---

## Task 13: `estimation/complementary_filter.c/h` — filter primitives (predict/correct)

**Files:**
- Create: `estimation/complementary_filter.h`, `estimation/complementary_filter.c`
- Test: `tests/test_complementary_filter.c`

**Interfaces:**
- Consumes: `dt_is_valid` (Task 1).
- Produces: `typedef struct { double v_est_mps; double alpha; } complementary_filter_t;`,
  `void complementary_filter_init(complementary_filter_t *f, double alpha)`,
  `void complementary_filter_predict(complementary_filter_t *f, double a_longitudinal_mps2, double dt)`,
  `void complementary_filter_correct(complementary_filter_t *f, double v_tof_mps)`.
  Consumed by `estimation/estimator.c` (Task 14), which is the only caller
  — it decides, per docs/design.md §6.5, when to call `correct` (only on
  ticks with a freshly computed, non-stale RAW speed).

- [ ] **Step 1: Write `estimation/complementary_filter.h`**

```c
#ifndef ESTIMATION_COMPLEMENTARY_FILTER_H
#define ESTIMATION_COMPLEMENTARY_FILTER_H

/* V1 complementary filter (docs/design.md §6.5). A Kalman filter is an
   explicit, named V2 possibility (DEBT-2) — not implemented here.
   The IMU provides only short-term propagation between valid ToF
   corrections; this filter does not rely on IMU integration for long-term
   accuracy — every correct() call pulls the estimate back toward the
   ToF-derived ground truth. */
typedef struct {
    double v_est_mps;
    double alpha; /* weight on IMU-propagated prediction at each correction */
} complementary_filter_t;

void complementary_filter_init(complementary_filter_t *f, double alpha);

/* v_pred = v_est + a_longitudinal*dt. On invalid dt (docs/design.md §6.4),
   holds v_est_mps unchanged (zero propagation) instead of integrating
   over a bad dt. Call once per IMU tick. */
void complementary_filter_predict(complementary_filter_t *f, double a_longitudinal_mps2, double dt);

/* v_est = alpha*v_est + (1-alpha)*v_tof. Call only on ticks with a fresh
   (non-stale) ToF-derived speed, AFTER predict() has run for this tick. */
void complementary_filter_correct(complementary_filter_t *f, double v_tof_mps);

#endif /* ESTIMATION_COMPLEMENTARY_FILTER_H */
```

- [ ] **Step 2: Write `estimation/complementary_filter.c`**

```c
#include "estimation/complementary_filter.h"
#include "common/dt_validation.h"

void complementary_filter_init(complementary_filter_t *f, double alpha) {
    f->v_est_mps = 0.0;
    f->alpha = alpha;
}

void complementary_filter_predict(complementary_filter_t *f, double a_longitudinal_mps2, double dt) {
    if (!dt_is_valid(dt)) {
        return; /* hold last state */
    }
    f->v_est_mps = f->v_est_mps + a_longitudinal_mps2 * dt;
}

void complementary_filter_correct(complementary_filter_t *f, double v_tof_mps) {
    f->v_est_mps = f->alpha * f->v_est_mps + (1.0 - f->alpha) * v_tof_mps;
}
```

- [ ] **Step 3: Write `tests/test_complementary_filter.c`**

```c
#include "tests/test_framework.h"
#include "estimation/complementary_filter.h"

TEST(test_init_starts_at_zero_velocity) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    CHECK_NEAR(f.v_est_mps, 0.0, 1e-12);
}

TEST(test_predict_integrates_accel_over_dt) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    complementary_filter_predict(&f, 0.5, 0.1);
    CHECK_NEAR(f.v_est_mps, 0.05, 1e-9);
}

TEST(test_predict_invalid_dt_holds_state) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    complementary_filter_predict(&f, 0.5, 0.1);
    double before = f.v_est_mps;
    complementary_filter_predict(&f, 0.5, -0.01);
    complementary_filter_predict(&f, 0.5, 3.0);
    CHECK_NEAR(f.v_est_mps, before, 1e-12);
}

TEST(test_correct_blends_prediction_toward_tof_speed) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    f.v_est_mps = 1.0; /* pretend prediction landed here */
    complementary_filter_correct(&f, 0.0); /* ToF says 0.0 */
    CHECK_NEAR(f.v_est_mps, 0.9 * 1.0 + 0.1 * 0.0, 1e-9);
}

TEST(test_alpha_one_ignores_tof_entirely) {
    complementary_filter_t f;
    complementary_filter_init(&f, 1.0);
    f.v_est_mps = 0.7;
    complementary_filter_correct(&f, 5.0);
    CHECK_NEAR(f.v_est_mps, 0.7, 1e-9);
}

int main(void) {
    RUN_TEST(test_init_starts_at_zero_velocity);
    RUN_TEST(test_predict_integrates_accel_over_dt);
    RUN_TEST(test_predict_invalid_dt_holds_state);
    RUN_TEST(test_correct_blends_prediction_toward_tof_speed);
    RUN_TEST(test_alpha_one_ignores_tof_entirely);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(complementary_filter_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/complementary_filter.c)
target_link_libraries(complementary_filter_lib common_lib)

add_executable(test_complementary_filter test_complementary_filter.c)
target_link_libraries(test_complementary_filter complementary_filter_lib)
add_test(NAME test_complementary_filter COMMAND test_complementary_filter)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add estimation/complementary_filter.h estimation/complementary_filter.c tests/test_complementary_filter.c tests/CMakeLists.txt
git commit -m "add: V1 complementary filter predict/correct primitives"
```

---

## Task 14: `estimation/estimator.c/h` — orchestrates the full estimation chain

**Files:**
- Create: `estimation/estimator.h`, `estimation/estimator.c`
- Test: `tests/test_estimator.c`

**Interfaces:**
- Consumes: `raw_speed_estimator_t`/`raw_speed_init`/`raw_speed_on_new_range`/`raw_speed_mark_stale`
  (Task 11), `orientation_1d_t`/`orientation_1d_init`/`orientation_1d_update`
  (Task 10), `gravity_compensate` (Task 9), `complementary_filter_t`/
  `complementary_filter_init`/`_predict`/`_correct` (Task 13),
  `dt_is_valid`/`dt_between` (Task 1), `range_sample_t`/`imu_sample_t`
  (Task 2).
- Produces: `typedef struct { raw_speed_t raw_speed; double fused_speed_mps; double theta_rad; } estimator_output_t;`,
  `typedef struct { raw_speed_estimator_t raw; orientation_1d_t orientation; complementary_filter_t comp; timestamp_t last_imu_ts; bool has_prev_imu_ts; } estimator_t;`,
  `void estimator_init(estimator_t *e, double alpha)`,
  `estimator_output_t estimator_tick(estimator_t *e, const range_sample_t *range, const imu_sample_t *imu)`.
  This is *the* estimation chain (docs/design.md §4, left column) as one
  callable unit — consumed by `main_host_sim.c` (Task 21) and the
  measurement-contract test (Task 15).

- [ ] **Step 1: Write `estimation/estimator.h`**

```c
#ifndef ESTIMATION_ESTIMATOR_H
#define ESTIMATION_ESTIMATOR_H

#include "common/types.h"
#include "common/measurement.h"
#include "estimation/raw_speed.h"
#include "estimation/orientation_1d.h"
#include "estimation/complementary_filter.h"

typedef struct {
    raw_speed_estimator_t raw;
    orientation_1d_t orientation;
    complementary_filter_t comp;
    timestamp_t last_imu_ts;
    bool has_prev_imu_ts;
} estimator_t;

typedef struct {
    raw_speed_t raw_speed;
    double fused_speed_mps;
    double theta_rad;
} estimator_output_t;

void estimator_init(estimator_t *e, double alpha);

/* Call once per main-loop tick with this tick's range/IMU read results
   (status HAL_OK/HAL_NO_NEW_DATA/HAL_FAULT per docs/design.md §5.1-5.2).
   Implements docs/design.md §4/§6 end to end: IMU propagation every tick
   (dt-validated), ToF-driven RAW differencing + correction only on a
   fresh sample, held+flagged-stale otherwise. */
estimator_output_t estimator_tick(estimator_t *e, const range_sample_t *range, const imu_sample_t *imu);

#endif /* ESTIMATION_ESTIMATOR_H */
```

- [ ] **Step 2: Write `estimation/estimator.c`**

```c
#include "estimation/estimator.h"
#include "estimation/gravity_compensation.h"
#include "common/dt_validation.h"

void estimator_init(estimator_t *e, double alpha) {
    raw_speed_init(&e->raw);
    orientation_1d_init(&e->orientation);
    complementary_filter_init(&e->comp, alpha);
    e->last_imu_ts.t_s = 0.0;
    e->has_prev_imu_ts = false;
}

estimator_output_t estimator_tick(estimator_t *e, const range_sample_t *range, const imu_sample_t *imu) {
    if (imu->status == HAL_OK) {
        if (e->has_prev_imu_ts) {
            double dt = dt_between(e->last_imu_ts, imu->ts);
            if (dt_is_valid(dt)) {
                orientation_1d_update(&e->orientation, imu->gyro_rps[1], dt);
                double a_long = gravity_compensate(imu->accel_mps2[0], imu->accel_mps2[2], e->orientation.theta_rad);
                complementary_filter_predict(&e->comp, a_long, dt);
            }
            /* invalid dt: orientation_1d_update/complementary_filter_predict
               are not called, holding both states per docs/design.md §6.4 */
        }
        e->last_imu_ts = imu->ts;
        e->has_prev_imu_ts = true;
    }
    /* HAL_NO_NEW_DATA / HAL_FAULT: no propagation this tick, v_est held */

    if (range->status == HAL_OK) {
        raw_speed_on_new_range(&e->raw, range->range_m, range->ts);
        if (!e->raw.speed.is_stale) {
            complementary_filter_correct(&e->comp, e->raw.speed.value_mps);
        }
    } else {
        raw_speed_mark_stale(&e->raw);
    }

    estimator_output_t out;
    out.raw_speed = e->raw.speed;
    out.fused_speed_mps = e->comp.v_est_mps;
    out.theta_rad = e->orientation.theta_rad;
    return out;
}
```

- [ ] **Step 3: Write `tests/test_estimator.c`**

```c
#include "tests/test_framework.h"
#include "estimation/estimator.h"

static imu_sample_t make_imu(double t, double a_x, double a_z, double gyro_y, hal_status_t status) {
    imu_sample_t s;
    s.accel_mps2[0] = a_x; s.accel_mps2[1] = 0; s.accel_mps2[2] = a_z;
    s.gyro_rps[0] = 0; s.gyro_rps[1] = gyro_y; s.gyro_rps[2] = 0;
    s.ts.t_s = t;
    s.status = status;
    return s;
}

static range_sample_t make_range(double t, double r, hal_status_t status) {
    range_sample_t s;
    s.range_m = r;
    s.ts.t_s = t;
    s.status = status;
    return s;
}

TEST(test_first_tick_produces_stale_raw_and_zero_fused) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r = make_range(0.0, 1.0, HAL_OK);
    estimator_output_t out = estimator_tick(&e, &r, &imu);
    CHECK(out.raw_speed.is_stale);
    CHECK_NEAR(out.fused_speed_mps, 0.0, 1e-9);
}

TEST(test_no_new_tof_holds_raw_stale_but_imu_still_propagates) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.01, 0.5, 9.81, 0.0, HAL_OK); /* level, 0.5 m/s^2 forward */
    range_sample_t r1 = make_range(0.01, 1.0, HAL_NO_NEW_DATA);
    estimator_output_t out = estimator_tick(&e, &r1, &imu1);
    CHECK(out.raw_speed.is_stale);
    CHECK(out.fused_speed_mps > 0.0); /* IMU propagated velocity forward */
}

TEST(test_fresh_tof_sample_corrects_fused_toward_raw) {
    estimator_t e;
    estimator_init(&e, 0.5); /* heavy weight on ToF correction to make the test's expected pull visible */
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.05, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r1 = make_range(0.05, 0.95, HAL_OK); /* -0.05m over 0.05s -> raw = -1.0 (moving away) is wrong sign; use approach */
    range_sample_t r1b = make_range(0.05, 0.9, HAL_OK); /* -0.1m over 0.05s -> raw = +2.0 m/s toward target */
    estimator_output_t out = estimator_tick(&e, &r1b, &imu1);
    CHECK(!out.raw_speed.is_stale);
    CHECK_NEAR(out.raw_speed.value_mps, 2.0, 1e-9);
    /* v_pred stayed ~0 (level, no accel), corrected toward 2.0 with alpha=0.5 */
    CHECK_NEAR(out.fused_speed_mps, 0.5 * 0.0 + 0.5 * 2.0, 1e-9);
    (void)r1;
}

TEST(test_imu_fault_holds_fused_speed) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.01, 0.5, 9.81, 0.0, HAL_FAULT);
    range_sample_t r1 = make_range(0.01, 1.0, HAL_NO_NEW_DATA);
    estimator_output_t out = estimator_tick(&e, &r1, &imu1);
    CHECK_NEAR(out.fused_speed_mps, 0.0, 1e-9); /* no propagation happened */
}

int main(void) {
    RUN_TEST(test_first_tick_produces_stale_raw_and_zero_fused);
    RUN_TEST(test_no_new_tof_holds_raw_stale_but_imu_still_propagates);
    RUN_TEST(test_fresh_tof_sample_corrects_fused_toward_raw);
    RUN_TEST(test_imu_fault_holds_fused_speed);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(estimator_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/estimator.c)
target_link_libraries(estimator_lib raw_speed_lib orientation_1d_lib gravity_compensation_lib complementary_filter_lib common_lib m)

add_executable(test_estimator test_estimator.c)
target_link_libraries(test_estimator estimator_lib m)
add_test(NAME test_estimator COMMAND test_estimator)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add estimation/estimator.h estimation/estimator.c tests/test_estimator.c tests/CMakeLists.txt
git commit -m "add: estimation chain orchestrator (raw + gravity comp + 1D pitch + complementary filter)"
```

---

## Task 15: Estimation-chain measurement-contract integration test

**Files:**
- Test: `tests/test_estimation_chain_metrics.c`

**Interfaces:**
- Consumes: `hal_host_world_t`/`hal_host_world_init`/`hal_host_world_tick`/`hal_host_create`
  (Task 8), `estimator_t`/`estimator_init`/`estimator_tick` (Task 14),
  `metrics_accum_t`/`metrics_add`/`metrics_rmse`/`metrics_mean_bias`/`metrics_variance`,
  `find_settling_time` (Task 12). Ground truth (`w.cart.true_range_m`,
  `w.cart.true_velocity_mps`) is read directly off `hal_host_world_t`,
  never passed to `estimator_tick` — enforcing docs/design.md §8's "ground
  truth never fed to the estimator" rule structurally in this test.
- Produces: nothing new — this is the §8 estimation-chain evaluation
  contract exercised end to end, standing alone as a regression test for
  RMSE/bias/variance (RAW and FUSED), dropout recovery, and step response.

- [ ] **Step 1: Write `tests/test_estimation_chain_metrics.c`**

```c
#include "tests/test_framework.h"
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "common/metrics.h"
#include <stdio.h>

#define TICK_DT 0.01
#define N_TICKS 500 /* 5 seconds */

TEST(test_fused_rmse_bias_variance_under_nominal_noise) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 42);
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t raw_m, fused_m;
    metrics_init(&raw_m);
    metrics_init(&fused_m);

    double true_accel = 0.2; /* constant gentle push toward target */
    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs; s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);

        double true_v = w.cart.true_velocity_mps; /* ground truth, never fed to estimator_tick */
        if (!out.raw_speed.is_stale) metrics_add(&raw_m, out.raw_speed.value_mps, true_v);
        metrics_add(&fused_m, out.fused_speed_mps, true_v);
    }

    /* FUSED must track ground truth substantially better than RAW's raw
       differencing noise floor (the demonstrable point of fusion). */
    CHECK(metrics_rmse(&fused_m) < metrics_rmse(&raw_m));
    CHECK(metrics_rmse(&fused_m) < 0.15);
    CHECK(fabs(metrics_mean_bias(&fused_m)) < 0.1);
}

TEST(test_dropout_recovery_fused_reconverges_after_tof_gap) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 7);
    estimator_t e;
    estimator_init(&e, 0.90);

    double true_accel = 0.15;
    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        bool in_gap = (i >= 100 && i < 130); /* ~300ms ToF gap, well under DT_MAX_S=2.0 */
        if (in_gap) {
            /* force dropout on the world's range sensor directly */
        }
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        if (in_gap) w.range_sensor.dropout_active = true;
        else w.range_sensor.dropout_active = false;
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        t_s[i] = w.clock_now_s;
        fused[i] = out.fused_speed_mps;
    }
    double true_v_final = w.cart.true_velocity_mps;
    double settle = find_settling_time(t_s, fused, N_TICKS, true_v_final, 0.1);
    CHECK(settle >= 0.0); /* it does reconverge within the run */
}

TEST(test_response_lag_after_step_change_in_true_velocity) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 3);
    estimator_t e;
    estimator_init(&e, 0.90);

    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        double true_accel = (i < 200) ? 0.0 : 5.0; /* step in acceleration -> ramp to a new velocity */
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        t_s[i] = w.clock_now_s;
        fused[i] = out.fused_speed_mps;
    }
    double true_v_final = w.cart.true_velocity_mps;
    double settle = find_settling_time(t_s, fused, N_TICKS, true_v_final, 0.15);
    CHECK(settle >= 0.0);
    CHECK(settle < t_s[N_TICKS - 1]); /* settles before the run ends */
}

TEST(test_imu_bias_drift_stays_bounded_by_tof_corrections) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 9);
    sim_imu_set_accel_bias(&w.imu, 0.3); /* persistent IMU accel bias */
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t fused_m;
    metrics_init(&fused_m);
    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, 0.1, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        metrics_add(&fused_m, out.fused_speed_mps, w.cart.true_velocity_mps);
    }
    /* Bounded, not diverging without limit, thanks to periodic ToF
       correction — this is the point of fusion over pure IMU integration. */
    CHECK(metrics_rmse(&fused_m) < 0.5);
}

int main(void) {
    RUN_TEST(test_fused_rmse_bias_variance_under_nominal_noise);
    RUN_TEST(test_dropout_recovery_fused_reconverges_after_tof_gap);
    RUN_TEST(test_response_lag_after_step_change_in_true_velocity);
    RUN_TEST(test_imu_bias_drift_stays_bounded_by_tof_corrections);
    TEST_SUMMARY();
    return 0;
}
```

Note on Step 1: if the nominal-noise RMSE/bias thresholds don't hold on
first run against the actual noise profile from `hal_host.c` (Task 8),
tune thresholds (not the underlying filter) to match observed, sane
behavior — the point of this test is regression-locking real behavior, not
hitting arbitrarily chosen numbers. Record the actual observed values in
the task's commit message.

- [ ] **Step 2: Add to `tests/CMakeLists.txt`**

```cmake
add_executable(test_estimation_chain_metrics test_estimation_chain_metrics.c)
target_link_libraries(test_estimation_chain_metrics hal_host_lib estimator_lib metrics_lib m)
add_test(NAME test_estimation_chain_metrics COMMAND test_estimation_chain_metrics)
```

- [ ] **Step 3: Build, run, report pass count for all suites so far. Record observed RMSE/bias/variance/settling-time values.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add tests/test_estimation_chain_metrics.c tests/CMakeLists.txt
git commit -m "add: estimation chain measurement-contract integration test (RMSE/bias/variance/dropout/step-response/bias-drift)"
```

---

## Task 16: `guidance/v_safe.c/h` — range-gated speed limit

**Files:**
- Create: `guidance/v_safe.h`, `guidance/v_safe.c`
- Test: `tests/test_v_safe.c`

**Interfaces:**
- Consumes: `<math.h>` only.
- Produces: `double v_safe(double range_m)` using the module-internal
  constants `V_SAFE_K = 0.5`, `V_SAFE_CAP_MPS = 0.6`, `V_SAFE_FLOOR_MPS = 0.03`
  (docs/design.md §7.1). Consumed by `guidance/pd_controller.c` callers
  (the orchestrator in Task 21, and Task 19's control-chain test).

- [ ] **Step 1: Write `guidance/v_safe.h`**

```c
#ifndef GUIDANCE_V_SAFE_H
#define GUIDANCE_V_SAFE_H

/* v_safe(r) = clamp(k*sqrt(r), v_floor, v_cap) — the glideslope-style
   range-gated closing-speed limit (docs/design.md §7.1). */
double v_safe(double range_m);

#endif /* GUIDANCE_V_SAFE_H */
```

- [ ] **Step 2: Write `guidance/v_safe.c`**

```c
#include "guidance/v_safe.h"
#include <math.h>

#define V_SAFE_K 0.5
#define V_SAFE_CAP_MPS 0.6
#define V_SAFE_FLOOR_MPS 0.03

double v_safe(double range_m) {
    if (range_m < 0.0) range_m = 0.0;
    double v = V_SAFE_K * sqrt(range_m);
    if (v > V_SAFE_CAP_MPS) v = V_SAFE_CAP_MPS;
    if (v < V_SAFE_FLOOR_MPS) v = V_SAFE_FLOOR_MPS;
    return v;
}
```

- [ ] **Step 3: Write `tests/test_v_safe.c`**

```c
#include "tests/test_framework.h"
#include "guidance/v_safe.h"

TEST(test_v_safe_follows_sqrt_shape_in_midrange) {
    /* k*sqrt(0.36) = 0.5*0.6 = 0.3, below the 0.6 cap */
    CHECK_NEAR(v_safe(0.36), 0.3, 1e-9);
}

TEST(test_v_safe_capped_near_start_of_approach) {
    CHECK_NEAR(v_safe(1.0), 0.5, 1e-9); /* 0.5*sqrt(1)=0.5, under cap */
    CHECK_NEAR(v_safe(10.0), 0.6, 1e-9); /* 0.5*sqrt(10)=1.58 -> capped at 0.6 */
}

TEST(test_v_safe_floored_near_contact) {
    CHECK_NEAR(v_safe(0.0), 0.03, 1e-9);
    CHECK_NEAR(v_safe(0.0001), 0.03, 1e-9); /* 0.5*sqrt(0.0001)=0.005 -> floored */
}

TEST(test_v_safe_negative_range_treated_as_zero) {
    CHECK_NEAR(v_safe(-0.5), 0.03, 1e-9);
}

int main(void) {
    RUN_TEST(test_v_safe_follows_sqrt_shape_in_midrange);
    RUN_TEST(test_v_safe_capped_near_start_of_approach);
    RUN_TEST(test_v_safe_floored_near_contact);
    RUN_TEST(test_v_safe_negative_range_treated_as_zero);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(v_safe_lib STATIC ${CMAKE_SOURCE_DIR}/guidance/v_safe.c)
target_link_libraries(v_safe_lib m)

add_executable(test_v_safe test_v_safe.c)
target_link_libraries(test_v_safe v_safe_lib m)
add_test(NAME test_v_safe COMMAND test_v_safe)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add guidance/v_safe.h guidance/v_safe.c tests/test_v_safe.c tests/CMakeLists.txt
git commit -m "add: v_safe(range) glideslope-style speed limit"
```

---

## Task 17: `guidance/pd_controller.c/h` — PD controller with filtered D-term and saturation

**Files:**
- Create: `guidance/pd_controller.h`, `guidance/pd_controller.c`
- Test: `tests/test_pd_controller.c`

**Interfaces:**
- Consumes: `dt_is_valid` (Task 1).
- Produces: `typedef struct { double kp; double kd; double tau_d; double prev_error; bool has_prev_error; double d_filtered; } pd_controller_t;`,
  `void pd_controller_init(pd_controller_t *c, double kp, double kd, double tau_d)`,
  `typedef struct { double control_output_unfiltered; double control_output_filtered; } pd_output_t;`,
  `pd_output_t pd_controller_update(pd_controller_t *c, double speed_error, double dt)`.
  The controller works in `speed_error` only — no servo/degrees knowledge
  (docs/design.md §7.2). Both filtered and unfiltered outputs are returned
  every call so the D-term noise-amplification comparison (docs/design.md
  §8) never needs a second run. Consumed by Task 21 (orchestrator) and
  Task 19 (control-chain measurement-contract test).

- [ ] **Step 1: Write `guidance/pd_controller.h`**

```c
#ifndef GUIDANCE_PD_CONTROLLER_H
#define GUIDANCE_PD_CONTROLLER_H

#include <stdbool.h>

/* Never knows about servo angles/degrees — output is normalized to
   [-1,+1] only (docs/design.md §7.2). Servo mapping lives in
   actuator_mapping.c. */
typedef struct {
    double kp;
    double kd;
    double tau_d; /* D-term low-pass time constant, seconds */
    double prev_error;
    bool has_prev_error;
    double d_filtered;
} pd_controller_t;

typedef struct {
    double control_output_unfiltered; /* saturated P + raw D, for comparison only */
    double control_output_filtered;   /* saturated P + low-pass-filtered D — the real output */
} pd_output_t;

void pd_controller_init(pd_controller_t *c, double kp, double kd, double tau_d);

/* On invalid dt (docs/design.md §6.4), the derivative term contributes 0
   for that tick (falls back to proportional-only) rather than dividing by
   a bad dt. */
pd_output_t pd_controller_update(pd_controller_t *c, double speed_error, double dt);

#endif /* GUIDANCE_PD_CONTROLLER_H */
```

- [ ] **Step 2: Write `guidance/pd_controller.c`**

```c
#include "guidance/pd_controller.h"
#include "common/dt_validation.h"

static double clamp11(double x) {
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

void pd_controller_init(pd_controller_t *c, double kp, double kd, double tau_d) {
    c->kp = kp;
    c->kd = kd;
    c->tau_d = tau_d;
    c->prev_error = 0.0;
    c->has_prev_error = false;
    c->d_filtered = 0.0;
}

pd_output_t pd_controller_update(pd_controller_t *c, double speed_error, double dt) {
    double d_raw = 0.0;
    bool have_derivative = false;

    if (c->has_prev_error && dt_is_valid(dt)) {
        d_raw = (speed_error - c->prev_error) / dt;
        have_derivative = true;
        double ema_alpha = dt / (c->tau_d + dt);
        c->d_filtered = ema_alpha * d_raw + (1.0 - ema_alpha) * c->d_filtered;
    }
    /* invalid dt or no previous error yet: d_raw stays 0, d_filtered holds */

    c->prev_error = speed_error;
    c->has_prev_error = true;

    double p_term = c->kp * speed_error;
    pd_output_t out;
    out.control_output_unfiltered = clamp11(p_term + (have_derivative ? c->kd * d_raw : 0.0));
    out.control_output_filtered = clamp11(p_term + c->kd * c->d_filtered);
    return out;
}
```

- [ ] **Step 3: Write `tests/test_pd_controller.c`**

```c
#include "tests/test_framework.h"
#include "guidance/pd_controller.h"
#include <math.h>

TEST(test_first_call_has_no_derivative_contribution) {
    pd_controller_t c;
    pd_controller_init(&c, 1.0, 1.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 0.2, 0.01);
    CHECK_NEAR(out.control_output_unfiltered, 0.2, 1e-9); /* pure P term */
}

TEST(test_proportional_term_scales_with_kp) {
    pd_controller_t c;
    pd_controller_init(&c, 2.0, 0.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 0.1, 0.01);
    CHECK_NEAR(out.control_output_filtered, 0.2, 1e-9);
}

TEST(test_output_saturates_to_plus_minus_one) {
    pd_controller_t c;
    pd_controller_init(&c, 10.0, 0.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 5.0, 0.01);
    CHECK_NEAR(out.control_output_filtered, 1.0, 1e-9);
    pd_output_t out2 = pd_controller_update(&c, -5.0, 0.01);
    CHECK_NEAR(out2.control_output_filtered, -1.0, 1e-9);
}

TEST(test_invalid_dt_falls_back_to_proportional_only_derivative) {
    pd_controller_t c;
    pd_controller_init(&c, 1.0, 5.0, 0.05);
    pd_controller_update(&c, 0.0, 0.01);
    pd_output_t out = pd_controller_update(&c, 1.0, 0.0); /* dt=0, invalid */
    CHECK_NEAR(out.control_output_unfiltered, 1.0, 1e-9); /* kp*1.0, no D term */
}

TEST(test_unfiltered_d_term_amplifies_noise_more_than_filtered) {
    pd_controller_t c;
    pd_controller_init(&c, 0.0, 1.0, 0.05); /* D-only, isolate D-term behavior */
    double errors[] = { 0.0, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5 }; /* noisy alternating error */
    double sum_sq_unfiltered = 0.0, sum_sq_filtered = 0.0;
    for (int i = 0; i < 8; i++) {
        pd_output_t out = pd_controller_update(&c, errors[i], 0.01);
        sum_sq_unfiltered += out.control_output_unfiltered * out.control_output_unfiltered;
        sum_sq_filtered += out.control_output_filtered * out.control_output_filtered;
    }
    CHECK(sum_sq_unfiltered > sum_sq_filtered); /* the demonstrable D-term amplification + fix */
}

int main(void) {
    RUN_TEST(test_first_call_has_no_derivative_contribution);
    RUN_TEST(test_proportional_term_scales_with_kp);
    RUN_TEST(test_output_saturates_to_plus_minus_one);
    RUN_TEST(test_invalid_dt_falls_back_to_proportional_only_derivative);
    RUN_TEST(test_unfiltered_d_term_amplifies_noise_more_than_filtered);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(pd_controller_lib STATIC ${CMAKE_SOURCE_DIR}/guidance/pd_controller.c)
target_link_libraries(pd_controller_lib common_lib)

add_executable(test_pd_controller test_pd_controller.c)
target_link_libraries(test_pd_controller pd_controller_lib m)
add_test(NAME test_pd_controller COMMAND test_pd_controller)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add guidance/pd_controller.h guidance/pd_controller.c tests/test_pd_controller.c tests/CMakeLists.txt
git commit -m "add: PD controller with filtered D-term, saturation, dt validation"
```

---

## Task 18: `guidance/actuator_mapping.c/h` — control_output to servo angle

**Files:**
- Create: `guidance/actuator_mapping.h`, `guidance/actuator_mapping.c`
- Test: `tests/test_actuator_mapping.c`

**Interfaces:**
- Consumes: nothing beyond plain doubles.
- Produces: `double actuator_map_to_servo_deg(double control_output)` using
  `SERVO_CENTER_DEG = 90.0`, `SERVO_HALF_RANGE_DEG = 45.0`
  (docs/design.md §7.3). Consumed by Task 21 (orchestrator) and Task 19.

- [ ] **Step 1: Write `guidance/actuator_mapping.h`**

```c
#ifndef GUIDANCE_ACTUATOR_MAPPING_H
#define GUIDANCE_ACTUATOR_MAPPING_H

/* This servo acts as a visual glideslope-error gauge, not a physical
   actuator — the cart is hand-pushed and the servo has no authority over
   its motion (docs/design.md §7.3, §9). */
double actuator_map_to_servo_deg(double control_output);

#endif /* GUIDANCE_ACTUATOR_MAPPING_H */
```

- [ ] **Step 2: Write `guidance/actuator_mapping.c`**

```c
#include "guidance/actuator_mapping.h"

#define SERVO_CENTER_DEG 90.0
#define SERVO_HALF_RANGE_DEG 45.0

double actuator_map_to_servo_deg(double control_output) {
    if (control_output > 1.0) control_output = 1.0;
    if (control_output < -1.0) control_output = -1.0;
    return SERVO_CENTER_DEG + control_output * SERVO_HALF_RANGE_DEG;
}
```

- [ ] **Step 3: Write `tests/test_actuator_mapping.c`**

```c
#include "tests/test_framework.h"
#include "guidance/actuator_mapping.h"

TEST(test_zero_control_output_maps_to_center) {
    CHECK_NEAR(actuator_map_to_servo_deg(0.0), 90.0, 1e-9);
}

TEST(test_plus_one_maps_to_upper_bound) {
    CHECK_NEAR(actuator_map_to_servo_deg(1.0), 135.0, 1e-9);
}

TEST(test_minus_one_maps_to_lower_bound) {
    CHECK_NEAR(actuator_map_to_servo_deg(-1.0), 45.0, 1e-9);
}

TEST(test_out_of_range_input_is_clamped_before_mapping) {
    CHECK_NEAR(actuator_map_to_servo_deg(2.0), 135.0, 1e-9);
    CHECK_NEAR(actuator_map_to_servo_deg(-2.0), 45.0, 1e-9);
}

int main(void) {
    RUN_TEST(test_zero_control_output_maps_to_center);
    RUN_TEST(test_plus_one_maps_to_upper_bound);
    RUN_TEST(test_minus_one_maps_to_lower_bound);
    RUN_TEST(test_out_of_range_input_is_clamped_before_mapping);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(actuator_mapping_lib STATIC ${CMAKE_SOURCE_DIR}/guidance/actuator_mapping.c)

add_executable(test_actuator_mapping test_actuator_mapping.c)
target_link_libraries(test_actuator_mapping actuator_mapping_lib)
add_test(NAME test_actuator_mapping COMMAND test_actuator_mapping)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add guidance/actuator_mapping.h guidance/actuator_mapping.c tests/test_actuator_mapping.c tests/CMakeLists.txt
git commit -m "add: actuator mapping (control_output -> servo angle), glideslope-gauge module"
```

---

## Task 19: Control-chain measurement-contract integration test

**Files:**
- Test: `tests/test_control_chain_metrics.c`

**Interfaces:**
- Consumes: `hal_host_world_t` (Task 8), `estimator_t`/`estimator_tick`
  (Task 14), `v_safe` (Task 16), `pd_controller_t`/`pd_controller_update`
  (Task 17), `actuator_map_to_servo_deg` (Task 18), `metrics_accum_t`/
  `jitter_accum_t`/`find_settling_time` (Task 12).
- Produces: nothing new — exercises docs/design.md §8's control-chain
  contract: RMS tracking error vs. `v_safe`, `control_output` variance,
  jitter pre/post D-term filtering, saturation percentage, step response.

- [ ] **Step 1: Write `tests/test_control_chain_metrics.c`**

```c
#include "tests/test_framework.h"
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "common/metrics.h"
#include <math.h>

#define TICK_DT 0.01
#define N_TICKS 500

TEST(test_rms_tracking_error_and_saturation_percentage) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 11);
    estimator_t e;
    estimator_init(&e, 0.90);
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    metrics_accum_t tracking_m, control_output_m;
    metrics_init(&tracking_m);
    metrics_init(&control_output_m);
    int saturated_ticks = 0;

    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, 0.2, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t est = estimator_tick(&e, &r, &s);

        double target = v_safe(w.cart.true_range_m);
        double speed_error = est.fused_speed_mps - target;
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);

        metrics_add(&tracking_m, est.fused_speed_mps, target);
        metrics_add(&control_output_m, ctrl.control_output_filtered, 0.0);
        if (fabs(ctrl.control_output_filtered) >= 0.999) saturated_ticks++;
    }

    double saturation_pct = 100.0 * saturated_ticks / N_TICKS;
    CHECK(metrics_rmse(&tracking_m) < 0.5);
    CHECK(saturation_pct >= 0.0 && saturation_pct <= 100.0);
}

TEST(test_derivative_jitter_reduced_by_filtering_on_noisy_run) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 13);
    estimator_t e;
    estimator_init(&e, 0.90);
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.5, 0.05); /* higher Kd to make jitter visible */

    jitter_accum_t j_unfiltered, j_filtered;
    jitter_init(&j_unfiltered);
    jitter_init(&j_filtered);

    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, 0.2, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t est = estimator_tick(&e, &r, &s);
        double speed_error = est.fused_speed_mps - v_safe(w.cart.true_range_m);
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        jitter_add(&j_unfiltered, ctrl.control_output_unfiltered);
        jitter_add(&j_filtered, ctrl.control_output_filtered);
    }
    /* This is the demonstrable point: unfiltered D-term jitters more than
       filtered, on the same run (docs/design.md §7.2, §8). */
    CHECK(jitter_rms(&j_unfiltered) > jitter_rms(&j_filtered));
}

TEST(test_step_response_time_to_speed_error_step) {
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);
    double t_s[N_TICKS]; double output[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        double speed_error = (i < 100) ? 0.0 : 0.3; /* step in error at t=1.0s */
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        t_s[i] = i * TICK_DT;
        output[i] = ctrl.control_output_filtered;
    }
    double final_output = output[N_TICKS - 1];
    double settle = find_settling_time(t_s, output, N_TICKS, final_output, 0.05);
    CHECK(settle >= 1.0); /* can't settle before the step happens */
    CHECK(settle < t_s[N_TICKS - 1]);
}

int main(void) {
    RUN_TEST(test_rms_tracking_error_and_saturation_percentage);
    RUN_TEST(test_derivative_jitter_reduced_by_filtering_on_noisy_run);
    RUN_TEST(test_step_response_time_to_speed_error_step);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 2: Add to `tests/CMakeLists.txt`**

```cmake
add_executable(test_control_chain_metrics test_control_chain_metrics.c)
target_link_libraries(test_control_chain_metrics hal_host_lib estimator_lib v_safe_lib pd_controller_lib actuator_mapping_lib metrics_lib m)
add_test(NAME test_control_chain_metrics COMMAND test_control_chain_metrics)
```

- [ ] **Step 3: Build, run, report pass count for all suites so far. Record observed metric values.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add tests/test_control_chain_metrics.c tests/CMakeLists.txt
git commit -m "add: control chain measurement-contract integration test (tracking/variance/jitter/saturation/step-response)"
```

---

## Task 20: `logging/csv_logger.c/h`

**Files:**
- Create: `logging/csv_logger.h`, `logging/csv_logger.c`
- Test: `tests/test_csv_logger.c`

**Interfaces:**
- Consumes: `<stdio.h>` only.
- Produces: `typedef struct { FILE *fp; } csv_logger_t;`,
  `bool csv_logger_open(csv_logger_t *log, const char *path)` (writes the
  header row immediately), `void csv_logger_write_row(csv_logger_t *log, double t_s, double true_range_m, double true_velocity_mps, double raw_speed_mps, bool raw_is_stale, double fused_speed_mps, double v_safe_mps, double speed_error_mps, double control_output_unfiltered, double control_output_filtered, double servo_angle_deg)`,
  `void csv_logger_close(csv_logger_t *log)`. Column order and names exactly
  match docs/design.md §10. Consumed by `main_host_sim.c` (Task 21).

- [ ] **Step 1: Write `logging/csv_logger.h`**

```c
#ifndef LOGGING_CSV_LOGGER_H
#define LOGGING_CSV_LOGGER_H

#include <stdio.h>
#include <stdbool.h>

typedef struct {
    FILE *fp;
} csv_logger_t;

/* Opens path for writing and writes the header row immediately. Returns
   false on failure to open (fp left NULL). */
bool csv_logger_open(csv_logger_t *log, const char *path);

/* Column order fixed by docs/design.md §10 — do not reorder without
   updating the design doc and README together. */
void csv_logger_write_row(csv_logger_t *log,
                           double t_s,
                           double true_range_m,
                           double true_velocity_mps,
                           double raw_speed_mps,
                           bool raw_is_stale,
                           double fused_speed_mps,
                           double v_safe_mps,
                           double speed_error_mps,
                           double control_output_unfiltered,
                           double control_output_filtered,
                           double servo_angle_deg);

void csv_logger_close(csv_logger_t *log);

#endif /* LOGGING_CSV_LOGGER_H */
```

- [ ] **Step 2: Write `logging/csv_logger.c`**

```c
#include "logging/csv_logger.h"

bool csv_logger_open(csv_logger_t *log, const char *path) {
    log->fp = fopen(path, "w");
    if (!log->fp) return false;
    fprintf(log->fp,
        "t_s,true_range_m,true_velocity_mps,raw_speed_mps,raw_is_stale,"
        "fused_speed_mps,v_safe_mps,speed_error_mps,"
        "control_output_unfiltered,control_output_filtered,servo_angle_deg\n");
    return true;
}

void csv_logger_write_row(csv_logger_t *log,
                           double t_s,
                           double true_range_m,
                           double true_velocity_mps,
                           double raw_speed_mps,
                           bool raw_is_stale,
                           double fused_speed_mps,
                           double v_safe_mps,
                           double speed_error_mps,
                           double control_output_unfiltered,
                           double control_output_filtered,
                           double servo_angle_deg) {
    if (!log->fp) return;
    fprintf(log->fp, "%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f\n",
        t_s, true_range_m, true_velocity_mps, raw_speed_mps, raw_is_stale ? 1 : 0,
        fused_speed_mps, v_safe_mps, speed_error_mps,
        control_output_unfiltered, control_output_filtered, servo_angle_deg);
}

void csv_logger_close(csv_logger_t *log) {
    if (log->fp) {
        fclose(log->fp);
        log->fp = NULL;
    }
}
```

- [ ] **Step 3: Write `tests/test_csv_logger.c`**

```c
#include "tests/test_framework.h"
#include "logging/csv_logger.h"
#include <string.h>

TEST(test_open_writes_header_row) {
    csv_logger_t log;
    CHECK(csv_logger_open(&log, "test_output_header.csv"));
    csv_logger_close(&log);
    FILE *f = fopen("test_output_header.csv", "r");
    CHECK(f != NULL);
    char line[512];
    fgets(line, sizeof(line), f);
    CHECK(strstr(line, "t_s") != NULL);
    CHECK(strstr(line, "raw_is_stale") != NULL);
    CHECK(strstr(line, "servo_angle_deg") != NULL);
    fclose(f);
}

TEST(test_write_row_produces_expected_column_count) {
    csv_logger_t log;
    csv_logger_open(&log, "test_output_row.csv");
    csv_logger_write_row(&log, 0.01, 1.0, 0.1, 0.1, false, 0.1, 0.5, -0.4, -0.6, -0.5, 60.0);
    csv_logger_close(&log);
    FILE *f = fopen("test_output_row.csv", "r");
    char header[512], row[512];
    fgets(header, sizeof(header), f);
    fgets(row, sizeof(row), f);
    int commas = 0;
    for (char *p = row; *p; p++) if (*p == ',') commas++;
    CHECK(commas == 10); /* 11 columns -> 10 commas */
    fclose(f);
}

int main(void) {
    RUN_TEST(test_open_writes_header_row);
    RUN_TEST(test_write_row_produces_expected_column_count);
    TEST_SUMMARY();
    return 0;
}
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`**

```cmake
add_library(csv_logger_lib STATIC ${CMAKE_SOURCE_DIR}/logging/csv_logger.c)

add_executable(test_csv_logger test_csv_logger.c)
target_link_libraries(test_csv_logger csv_logger_lib)
add_test(NAME test_csv_logger COMMAND test_csv_logger)
```

- [ ] **Step 5: Build, run, report pass count for all suites so far.**

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add logging/csv_logger.h logging/csv_logger.c tests/test_csv_logger.c tests/CMakeLists.txt
git commit -m "add: CSV logger matching docs/design.md §10 column contract"
```

---

## Task 21: `main_host_sim.c` — full pipeline wiring + demo scenario

**Files:**
- Create: `main_host_sim.c`
- Modify: `CMakeLists.txt` (add the executable target)

**Interfaces:**
- Consumes: everything from Tasks 8, 14, 16, 17, 18, 20 — this is the
  wiring point, not a new module with its own API.
- Produces: an executable `main_host_sim` that runs a scripted approach
  scenario for a fixed duration, drives `hal_host_world_tick` → HAL reads →
  `estimator_tick` → `v_safe`/`pd_controller_update`/`actuator_map_to_servo_deg`
  → `csv_logger_write_row`, once per tick, and writes `run_log.csv`.

- [ ] **Step 1: Write `main_host_sim.c`**

```c
#include <stdio.h>
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "logging/csv_logger.h"

#define TICK_DT 0.01
#define N_TICKS 1000 /* 10 seconds */
#define TRACK_LENGTH_M 1.0
#define INITIAL_RANGE_M 1.0

int main(void) {
    hal_host_world_t world;
    hal_host_world_init(&world, INITIAL_RANGE_M, TRACK_LENGTH_M, 1234);

    estimator_t estimator;
    estimator_init(&estimator, 0.90);

    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    csv_logger_t log;
    if (!csv_logger_open(&log, "run_log.csv")) {
        fprintf(stderr, "failed to open run_log.csv for writing\n");
        return 1;
    }

    for (int i = 0; i < N_TICKS; i++) {
        /* Scripted hand-push profile: accelerate toward the target for the
           first 3s, then coast/decelerate as v_safe(range) tightens. */
        double true_accel_mps2 = (i < 300) ? 0.3 : 0.0;

        hal_host_world_tick(&world, true_accel_mps2, 0.0, TICK_DT);
        hal_t h = hal_host_create(&world);

        range_sample_t range_sample;
        imu_sample_t imu_sample;
        hal_status_t range_status = h.range_read(h.ctx, &range_sample);
        hal_status_t imu_status = h.imu_read(h.ctx, &imu_sample);
        range_sample.status = range_status;
        imu_sample.status = imu_status;

        estimator_output_t est = estimator_tick(&estimator, &range_sample, &imu_sample);

        double target = v_safe(world.cart.true_range_m);
        double speed_error = est.fused_speed_mps - target;
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        double servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered);
        h.actuator_set_angle_deg(h.ctx, servo_deg);

        csv_logger_write_row(&log,
            world.clock_now_s,
            world.cart.true_range_m,
            world.cart.true_velocity_mps,
            est.raw_speed.value_mps,
            est.raw_speed.is_stale,
            est.fused_speed_mps,
            target,
            speed_error,
            ctrl.control_output_unfiltered,
            ctrl.control_output_filtered,
            servo_deg);
    }

    csv_logger_close(&log);
    printf("wrote run_log.csv (%d ticks, %.1fs)\n", N_TICKS, N_TICKS * TICK_DT);
    return 0;
}
```

- [ ] **Step 2: Add the executable target to top-level `CMakeLists.txt`**

```cmake
add_executable(main_host_sim
    main_host_sim.c
    common/dt_validation.c
    common/metrics.c
    sim/sim_noise.c
    sim/sim_cart.c
    sim/sim_range_sensor.c
    sim/sim_imu.c
    hal/hal_host.c
    estimation/gravity_compensation.c
    estimation/orientation_1d.c
    estimation/raw_speed.c
    estimation/complementary_filter.c
    estimation/estimator.c
    guidance/v_safe.c
    guidance/pd_controller.c
    guidance/actuator_mapping.c
    logging/csv_logger.c
)
target_link_libraries(main_host_sim m)
```

- [ ] **Step 3: Build and run it; confirm `run_log.csv` is produced with the expected 11 columns and 1000 data rows**

```bash
cmake --build build
./build/main_host_sim
head -3 run_log.csv
wc -l run_log.csv   # expect 1001 (header + 1000 rows)
```

- [ ] **Step 4: Run the full ctest suite once more; report the total pass count across every test binary.**

```bash
ctest --test-dir build --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add main_host_sim.c CMakeLists.txt
git commit -m "add: main_host_sim integration entry point, wires full V1 pipeline"
```

---

## Task 22: HAL boundary structural check

**Files:**
- Create: `tests/test_hal_boundary.sh`
- Modify: `tests/CMakeLists.txt` (register it as a ctest)

**Interfaces:**
- Consumes: nothing — a standalone shell script that greps the repo tree.
- Produces: a ctest entry that fails the build if any file other than
  `hal/hal_esp32.c` includes an ESP-IDF header. `hal/hal_esp32.c` does not
  exist yet in V1 (it's DEBT-1), so today this test simply proves the rule
  holds vacuously — it becomes load-bearing the moment DEBT-1 is picked up.

- [ ] **Step 1: Write `tests/test_hal_boundary.sh`**

```bash
#!/usr/bin/env bash
# Fails if any file other than hal/hal_esp32.c includes an ESP-IDF header.
# See CLAUDE.md constraint 1 / docs/design.md §2 item 3.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PATTERN='#include\s*[<"](driver/|esp_[a-z_]*\.h|freertos/|sdkconfig\.h)'

violations=$(grep -rlE "$PATTERN" \
    --include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' \
    . 2>/dev/null | grep -v '^\./hal/hal_esp32\.c$' || true)

if [ -n "$violations" ]; then
    echo "HAL boundary violation: ESP-IDF headers included outside hal/hal_esp32.c:"
    echo "$violations"
    exit 1
fi

echo "HAL boundary OK: no ESP-IDF includes outside hal/hal_esp32.c"
exit 0
```

- [ ] **Step 2: Make it executable and register it in `tests/CMakeLists.txt`**

```bash
chmod +x tests/test_hal_boundary.sh
```

```cmake
add_test(NAME test_hal_boundary COMMAND ${CMAKE_SOURCE_DIR}/tests/test_hal_boundary.sh)
```

- [ ] **Step 3: Run it directly to confirm it passes today, then via ctest**

```bash
./tests/test_hal_boundary.sh
cmake --build build && ctest --test-dir build --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add tests/test_hal_boundary.sh tests/CMakeLists.txt
git commit -m "add: structural check enforcing the hal_esp32.c ESP-IDF include boundary"
```

---

## Task 23: README

**Files:**
- Create: `README.md`

**Interfaces:**
- Consumes: the final observed test-suite pass count (Task 22, Step 3) and
  the metric values recorded in Tasks 15/19.
- Produces: the project's public-facing explanation, answering every
  question the user's spec requires verbatim (see Step 1 outline), plus
  conventions, HAL/sim separation, the development process, `v_safe`'s
  real-world glideslope analog, and full V1 measurement results.

- [ ] **Step 1: Write `README.md`** covering, at minimum, each of these as
  its own subsection with a direct plain-language answer (not just a
  pointer to docs/design.md — this file should be self-contained for a
  reader who never opens the design doc):
  - Why ToF dominates long-term velocity accuracy (periodic ground-truth
    correction bounds drift that IMU integration alone cannot).
  - Why the IMU helps between ToF updates, and why its role is deliberately
    limited to short-term propagation (docs/design.md §6.5).
  - Why differentiating ToF amplifies noise, and how staleness is handled
    between updates (docs/design.md §6.1).
  - Why orientation is 1D pitch only, not full 3D attitude (docs/design.md
    §3.1 — the rail constraint argument, restated in plain language).
  - Why the PD derivative term amplifies measurement noise and how that's
    addressed (docs/design.md §7.2 — the EMA filter, with the
    jitter-metric numbers from Task 19 quoted as evidence).
  - Why the estimator is timestamp-driven rather than fixed-dt, and what
    happens on invalid dt (docs/design.md §6.4).
  - Why the hardware boundary is isolated to one file (docs/design.md §2
    item 3 / CLAUDE.md constraint 1, and what `test_hal_boundary.sh`
    checks).
  - Why estimation and control chains are evaluated separately, never
    compared to each other directly (docs/design.md §4/§8).
  - What the servo physically represents (docs/design.md §9, quoted
    verbatim from `actuator_mapping.h`).
  - Coordinate/sign/unit conventions (docs/design.md §3, condensed table).
  - The HAL/sim separation and the development process agreements
    (condensed from docs/design.md §2, CLAUDE.md).
  - `v_safe(range)`'s shape and its real-world glideslope analog: cockpit
    glideslope indicators show deviation from a target closing profile
    without themselves flying the aircraft — same relationship the servo
    gauge has to this cart.
  - Full V1 measurement results: the actual RMSE/bias/variance numbers
    from Task 15, and RMS tracking error/variance/jitter-pre-post/
    saturation-%/step-response numbers from Task 19, both recorded as
    observed during those tasks — not fabricated placeholder numbers.
  - Build/test instructions (from CLAUDE.md's Build & test section).
  - Known debt (DEBT-1..4 table from docs/design.md §11) with an explicit
    "not started, requires confirmation" note for V2.

- [ ] **Step 2: Sanity-check the README's measurement numbers against the
  actual last `ctest --output-on-failure` run** — re-run tasks 15/19's
  binaries directly if the numbers weren't captured earlier, rather than
  reconstructing them from memory:

```bash
./build/test_estimation_chain_metrics
./build/test_control_chain_metrics
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "add: README covering conventions, architecture rationale, and V1 measurement results"
```

---

## Self-review notes

- **Spec coverage:** every §5 HAL interface (Task 3), every estimation
  module (§6, Tasks 9–14), every guidance module (§7, Tasks 16–18), both
  measurement-contract test suites (§8, Tasks 15/19), logging (§10, Task
  20), the HAL boundary structural check (§2 item 3, Task 22), and the
  README requirements (Task 23) each map to a task. `sim_noise`/`sim_cart`/
  `sim_range_sensor`/`sim_imu` (§2 item 2) are Tasks 4–7. dt validation
  (§6.4) is centralized in Task 1 and consumed by Tasks 10, 11, 13, 14, 17.
  1D-pitch justification (§3.1) lives in the design doc and is echoed in
  Task 10's header comment and required in the Task 23 README outline.
  RAW staleness CSV visibility (§6.1) is wired in Task 20/21
  (`raw_is_stale` column, populated from `est.raw_speed.is_stale`).
- **No V2 scope leakage:** no task references a Kalman filter
  implementation, ESP-IDF code, timing-jitter injection, or actuator
  deadband/rate-limiting as anything other than a DEBT-ID pointer.
- **Type consistency check:** `estimator_output_t`, `pd_output_t`,
  `hal_host_world_t`, and `metrics_accum_t`/`jitter_accum_t` field names
  are used identically across every task that references them (verified
  by re-reading each producing task's "Produces" line against every
  consuming task's code).

---

## Plan complete

All 23 tasks are specified end-to-end against `docs/design.md`. Ready to
execute.
