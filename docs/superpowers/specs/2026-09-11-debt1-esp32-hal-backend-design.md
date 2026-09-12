# DEBT-1: ESP32 hardware backend (`hal_esp32.c`) + real servo output — Design

Status: approved by project owner 2026-09-11, ready for implementation planning.

## 1. Purpose / scope

Implements DEBT-1 from `docs/design.md` §11: a real ESP32 HAL backend
(`hal_esp32.c`) driving the actual ToF sensor, IMU, and a servo, plus the
ESP-IDF app entry point (`main_esp32.c`) that wires it into the existing
pure-C99 estimation/guidance/logging pipeline — the same pipeline
`main_host_sim.c` already exercises against the host/sim backend.

Explicit non-goals (separate DEBT IDs, not touched here): Kalman filter
(DEBT-2), injected timing jitter/latency realism (DEBT-3), actuator
deadband/rate-limiting (DEBT-4).

Preconditions confirmed before starting: V1 is complete/tested/reviewed
(`CLAUDE.md` Status section); project owner gave explicit confirmation to
start DEBT-1 (`CLAUDE.md` constraint 9).

### 1.1 Known open physical-bring-up items (explicitly deferred, not blockers)

Per project owner decision:
- **IMU chip:** locked to ICM20948 only (no runtime auto-detect/dual-chip
  support). Simpler `hal_esp32.c` — one register map, no branch.
- **Axis/sign mapping:** `hardware_bringup` Test 2 (axis mapping) was never
  run. `hal_esp32.c`'s accel/gyro axis mapping is implemented as an
  **explicitly-marked placeholder** (see §3.2), to be corrected once Test 2
  is eventually run. This is a deliberate, documented risk accepted by the
  project owner — not an oversight.
- **Servo:** no servo hardware picked or wired yet. `hal_esp32.c`'s actuator
  backend uses standard hobby-servo defaults (see §3.3), called out as
  placeholder constants pending physical verification once hardware exists.

None of the above block writing/compiling this backend; they block trusting
its *real-world numeric output* until corrected against real hardware. Each
placeholder gets a loud, findable comment at its definition site.

## 2. File layout

New/changed paths:

```
firmware/                          NEW — separate ESP-IDF project (idf.py), not
                                    built by the host CMakeLists.txt.
  CMakeLists.txt                   NEW — standard idf.py project root.
  sdkconfig.defaults               NEW — target esp32.
  main/
    CMakeLists.txt                 NEW — idf_component_register: main_esp32.c +
                                    ../../common/*.c + ../../estimation/*.c +
                                    ../../guidance/*.c + ../../logging/*.c +
                                    ../../hal/hal_esp32.c + ../../hal/vl53l1x_status.c.
                                    INCLUDE_DIRS ../.. (repo root, so
                                    "common/types.h"-style includes resolve
                                    identically to the host build).
                                    REQUIRES vl53l1x_uld driver esp_timer
                                    esp_rom freertos.
    main_esp32.c                   NEW — pure C99, no ESP-IDF includes. Mirrors
                                    main_host_sim.c's loop body exactly.
  components/
    vl53l1x_uld/                   NEW — copy (not move/share) of the proven
                                    vendored ULD + platform shim from
                                    hardware_bringup/imu_bringup/components/
                                    vl53l1x_uld/. hardware_bringup keeps its own
                                    independent copy until that directory is
                                    deleted (still in active use: Test 2 not
                                    run, more Test 5 distances TBD).

hal/
  hal.h                            CHANGED — hal_t gains a 5th member:
                                    void (*pace_tick)(void *ctx);
  hal_host.c                       CHANGED — hal_host_create() sets pace_tick
                                    to a static no-op.
  hal_esp32.h                     NEW — hal_t hal_esp32_create(void); same
                                    interface shape as hal_host.h, zero ESP-IDF
                                    types re-exported.
  hal_esp32.c                     NEW — the ONLY new file with ESP-IDF
                                    #includes. Implements range_read (VL53L1X),
                                    imu_read (ICM20948), actuator_set_angle_deg
                                    (LEDC PWM servo), clock_now (esp_timer),
                                    pace_tick (busy-wait pacer).
  vl53l1x_status.h/.c              NEW — pure C99, NO ESP-IDF includes.
                                    hal_status_t vl53l1x_translate_status(bool
                                    data_ready, uint8_t range_status). Host-
                                    testable.

logging/
  csv_logger.h/.c                  CHANGED — add csv_logger_open_stream(
                                    csv_logger_t*, FILE*) and
                                    csv_logger_write_row_hw(...) (same columns
                                    minus true_range_m/true_velocity_mps, which
                                    have no hardware analog).

main_host_sim.c                   CHANGED — loop now also calls
                                    h.pace_tick(h.ctx) once per iteration (a
                                    no-op on this backend), so the loop body is
                                    byte-for-byte the same shape as
                                    main_esp32.c's, modulo the injected hal_t.

tests/
  test_vl53l1x_status.c           NEW — host unit test for the ToF status
                                    translator.
  CMakeLists.txt                  CHANGED — register the new test.
```

Nothing under `common/`, `estimation/`, `guidance/` changes — DEBT-1 is a
HAL/app-entry addition only, per constraint 2 (those directories stay pure
C99, zero hardware deps, untouched by this work).

## 3. `hal_esp32.c` backend behavior

### 3.1 ToF (`range_read`) — VL53L1X

Reuses the exact sequence already proven in `hardware_bringup`:
`VL53L1X_CheckForDataReady` → (if ready) `VL53L1X_GetDistance` +
`VL53L1X_GetRangeStatus` → `VL53L1X_ClearInterrupt` (acknowledges, sensor
auto-restarts in Timed/continuous mode). Continuous ranging, no interrupt
pin wired (matches the V1 polled-only design, `docs/design.md` §5.5).

Distance mode / timing budget / inter-measurement period constants carried
over unchanged from the bring-up firmware's confirmed values (short mode,
33 ms budget, 50 ms inter-measurement ≈ 20 Hz) — these were the subject of
bring-up Tests 4/5 and are not placeholders, they're measured-good.

Status translation is NOT a raw cast (constraint 7): `(data_ready,
range_status)` → `hal_status_t` goes through `vl53l1x_translate_status()`
in the new pure-C99 `hal/vl53l1x_status.c`:
- not ready → `HAL_NO_NEW_DATA`
- ready, `range_status == 0` (valid) → `HAL_OK`
- ready, any other `range_status` → `HAL_FAULT`

This is the one piece of real translation logic in this backend worth a
host-side unit test, and it gets one (`tests/test_vl53l1x_status.c`) — the
first host-testable fragment of the ESP32 backend, despite `hal_esp32.c`
itself being un-compilable on host.

### 3.2 IMU (`imu_read`) — ICM20948

Reuses the bring-up firmware's proven ICM20948 register init/read sequence
verbatim (bank-select dance, `PWR_MGMT_1`/`PWR_MGMT_2`, `±2g`/`±250dps`
config, burst register read starting at `0x2D`). I2C address `0x69` (a
`#define`, matching current bring-up wiring — trivially changed if `AD0`
is rewired to the chip's other default).

**Axis mapping placeholder (flagged per §1.1):** raw chip X/Y/Z accel and
gyro values are mapped into `accel_mps2[0]` (longitudinal), `accel_mps2[2]`
(vertical), `gyro_rps[1]` (pitch) using an identity mapping (chip X→HAL X,
chip Z→HAL Z, chip Y(gyro)→HAL pitch) with no sign correction, called out
in a header comment as **UNVERIFIED — Test 2 (axis/sign mapping) has never
been run; correct these indices/signs against real Test 2 results before
trusting this backend's estimator output.**

Every I2C read either succeeds or returns a non-`ESP_OK` `esp_err_t`; a
small named static translator (`translate_esp_err_to_hal_status`) maps
that to `HAL_OK`/`HAL_FAULT` (constraint 7 — explicit function, not a raw
cast, even though the mapping is a simple binary one). Per
`docs/design.md` §5.2, the IMU is fast enough that "new data every poll" is
the expected case — this backend always attempts a fresh register read on
every call and does not attempt to detect "no new sample since last read"
(no data-ready register is used), matching how the bring-up firmware
already treated it.

### 3.3 Actuator (`actuator_set_angle_deg`) — servo PWM

ESP-IDF `driver/ledc.h`. **Placeholder constants (flagged per §1.1),
standard hobby-servo defaults pending a real servo:** 50 Hz PWM, 500–2500 µs
pulse width mapped linearly across a configurable mechanical range
(default 0–180°), GPIO pin defaulted to a concrete, currently-unused pin
(`GPIO18` — free on the current wiring, distinct from the I2C pins 21/22
and UART0's default 1/3), as a single `#define` explicitly commented as a
placeholder to be changed at real wiring time if a different pin is more
convenient. Per
`docs/design.md` §5.3: if the incoming `angle_deg` is outside the
configured mechanical range, return `HAL_FAULT` without commanding the
PWM channel (checked inside the backend, not by the caller) — in practice
`actuator_mapping.c` already constrains output to 45–135°, well inside any
reasonable servo's range, so this is a defensive check, not a normally-hit
path.

### 3.4 Clock (`clock_now`) — `esp_timer_get_time()`

Converts microseconds to seconds (`double`), matching
`docs/design.md` §5.4's stated V2 approach exactly.

### 3.5 `pace_tick` — busy-wait pacer

Blocks until the next 100 Hz tick boundary using the exact
`esp_timer_get_time`-based remainder-of-period busy-wait already proven
reliable on this hardware/ESP-IDF combination in bring-up Tests 3/6
(deliberately NOT a `vTaskDelay`-based tick — bring-up found that pattern
triggers I2C software timeouts under sustained back-to-back transactions).
Internal "last iteration start" state is file-static in `hal_esp32.c`.

## 4. `hal_t` interface change

```c
typedef struct {
    void *ctx;
    hal_status_t (*range_read)(void *ctx, range_sample_t *out);
    hal_status_t (*imu_read)(void *ctx, imu_sample_t *out);
    hal_status_t (*actuator_set_angle_deg)(void *ctx, double angle_deg);
    timestamp_t (*clock_now)(void *ctx);
    void (*pace_tick)(void *ctx);   /* NEW */
} hal_t;
```

Verified: every existing `hal_t` in this repo is constructed exclusively
through `hal_host_create()` (grep confirms zero raw struct literals
elsewhere), so this is a single-point change plus each backend's
`_create()` populating the new member — no test file constructs `hal_t`
by hand and needs updating.

`hal_host_create()`'s `pace_tick` is a static no-op — the host/sim backend
already advances simulated time via the separate `hal_host_world_tick()`
call the test/main-loop driver makes, and has no wall clock to wait on.

Both `main_esp32.c` and `main_host_sim.c` call `h.pace_tick(h.ctx)` once per
loop iteration, making their loop bodies identical in shape — the HAL
injection point is the only difference between the two mains, which is the
literal point of the HAL boundary (`docs/design.md` §2 item 1/5).

## 5. `main_esp32.c`

Pure C99, **zero ESP-IDF `#include`s** (constraint 1 — `app_main` is just a
function name/signature ESP-IDF calls by convention; defining it requires
no ESP-IDF header). Structure, per iteration:

```
h.pace_tick(h.ctx)
now = h.clock_now(h.ctx)
range_status = h.range_read(h.ctx, &range_sample)
imu_status  = h.imu_read(h.ctx, &imu_sample)
est = estimator_tick(&estimator, &range_sample, &imu_sample)
target = v_safe(last_measured_range_m)
speed_error = est.fused_speed_mps - target
ctrl = pd_controller_update(&pd, speed_error, dt)
servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered)
h.actuator_set_angle_deg(h.ctx, servo_deg)
csv_logger_write_row_hw(&log, ...)
```

Runs forever (no `N_TICKS` bound, unlike the host demo runner). No
scripted acceleration profile (that was host-sim-only, simulating a
hand-push) — real IMU/ToF readings drive everything.

## 6. On-device logging

`logging/csv_logger.c` gains:
- `bool csv_logger_open_stream(csv_logger_t *log, FILE *stream)` — points
  `log->fp` at an already-open stream (e.g. `stdout`) and writes the
  hardware-variant header row. No `fopen`/filesystem dependency, so no
  SPIFFS/SD setup needed — `main_esp32.c` passes `stdout`, captured via
  `idf.py monitor`.
- `csv_logger_write_row_hw(csv_logger_t *log, double t_s, double
  measured_range_m, double raw_speed_mps, bool raw_is_stale, double
  fused_speed_mps, double v_safe_mps, double speed_error_mps, double
  control_output_unfiltered, double control_output_filtered, double
  servo_angle_deg)` — same column set as the sim variant minus
  `true_range_m`/`true_velocity_mps` (no hardware analog: those are
  simulation ground truth only, per `docs/design.md` §8), with
  `measured_range_m` (the real last-known ToF reading, not ground truth)
  in `true_range_m`'s old column position.

Both logger changes are additive; the existing `csv_logger_open`/
`csv_logger_write_row` (host/sim path) are unchanged.

## 7. Testing / gating

- Host-native suite (`ctest`) covers: the new `vl53l1x_status` translator
  unit test, and (implicitly, via existing tests) that `hal_t`'s new member
  doesn't break any existing host-backend construction or estimation/
  guidance/control test.
- `tests/test_hal_boundary.sh` must continue to pass unmodified — it
  already special-cases `hal/hal_esp32.c` as the sole permitted exception
  (written speculatively before this file existed), so this is a
  regression check, not a new mechanism.
- `firmware/`'s ESP-IDF project must build cleanly (`idf.py build`,
  target `esp32`) — this is compiled but **not flashed or run**, since no
  physical hardware is attached to this session. On-device verification
  (real IMU/ToF reads, servo movement) is an explicit manual follow-up for
  the project owner once hardware is wired; nothing in this work claims
  device-level success without that hardware-in-hand verification.
- After every implementation step: run the full host `ctest` suite and
  report the pass count (constraint 8) — applies throughout, same as any
  other V1/V2 work.

## 8. Explicit risks / follow-ups (not new DEBT IDs — tracked here)

- Axis/sign mapping placeholder (§3.2) must be corrected once bring-up
  Test 2 is run; until then, `hal_esp32.c`'s estimator output should not be
  trusted for sign correctness.
- Servo GPIO pin, pulse-width range, and mechanical range (§3.3) are
  defaults pending a real servo; must be verified/tuned once hardware
  exists.
- `hardware_bringup/imu_bringup/components/vl53l1x_uld/` and
  `firmware/components/vl53l1x_uld/` are now two independent copies of the
  same vendored source. This is accepted duplication (see §2) — no
  synchronization mechanism between them; `hardware_bringup/` is deleted
  once bring-up is complete, at which point the duplication resolves
  itself.
