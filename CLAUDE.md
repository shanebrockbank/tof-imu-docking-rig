# CLAUDE.md

Project: **tof-imu-docking-rig** — hand-pushed cart proximity-ops demo
(ToF + IMU → estimation → guidance/control → servo gauge) on ESP32,
architected HAL-first and fully host-simulatable.

**Full design spec:** `docs/design.md`. Read it before touching
`estimation/`, `guidance/`, `hal/`, or `sim/` — it defines units, sign
conventions, HAL interface contracts, filter equations, and the
measurement/evaluation contract. This file (`CLAUDE.md`) is the quick
operating reference; `docs/design.md` is authoritative on anything this
file doesn't cover.

## Hard constraints (do not violate)

1. **`hal/hal_esp32.c` is the only file in this repo allowed to `#include`
   any ESP-IDF header** (`driver/*.h`, `esp_*.h`, `freertos/*.h`, `sdkconfig.h`,
   etc.). `hal/hal_esp32.h` re-exports zero ESP-IDF types — its declared
   interface is identical in shape to `hal_host.h`. This is checked
   mechanically by `tests/test_hal_boundary.sh`, which greps every file
   except `hal_esp32.c` for ESP-IDF include patterns; a match fails the
   build.
   (The vendored third-party VL53L1X driver under `hal/hal_esp32.c`'s own
   `firmware/components/vl53l1x_uld/` — including its hand-written ESP-IDF
   platform shim — is separately excepted by that script, the same way
   `hardware_bringup/`'s pre-V1 firmware already is; this constraint's
   "only file" language refers to hand-written pipeline code.)
   If a change seems to require an ESP-IDF include outside that one
   file, stop and reconsider the design — don't add an exception.
2. **`estimation/`, `guidance/`, and `common/` are pure C99 with zero
   hardware dependencies of any kind** (no ESP-IDF, no host-only libc
   assumptions beyond `<math.h>`/`<stdbool.h>`/`<stdint.h>`/`<string.h>`).
   They interact with the world only through the `hal_t` function-pointer
   struct in `hal/hal.h`.
3. **`hal/` vs `sim/` stay separate.** `hal/` = interface shape only, no
   physics. `sim/` = the actual simulated physics/noise/dynamics.
   `hal_host.c` calls into `sim/`; it never contains simulation math
   itself.
4. **No dynamic allocation after init.** Filter/controller/logger state is
   fixed-size structs, stack- or static-allocated. One `.c`/`.h` pair per
   logical module.
5. **Never call estimation/guidance/control code from ISR context.** V1 is
   fully polled (no interrupts) — see `docs/design.md` §5.5/§2. If that
   ever changes: ISR → flag/queue only → polled read in the main loop.
6. **Don't call `control_output`/`servo_angle` a "speed estimate."** It's a
   control output derived from FUSED speed *and* guidance error — a
   different quantity from RAW/FUSED. See `docs/design.md` §4.
7. **Any two same-concept-different-values enums/types get an explicit
   translator function**, never a raw cast (e.g. HAL status codes vs. sim
   fault codes).
8. **After every implementation step, run the host test suite and report
   the pass count** (e.g. "42/42 tests passing"). A step isn't done until
   the gate is green.
9. **Don't start V2 items without explicit confirmation** that V1 is
   complete, tested, and reviewed. V2 candidates are tracked as DEBT-1..4
   in `docs/design.md` §11 — reference the ID, don't silently fold V2 scope
   into a V1 change.

## Directory layout

```
common/      Shared types (timestamp_t, hal_status_t, dt_is_valid()) — pure C99.
hal/         hal.h (interface structs), hal_host.c/.h (sim-backed), hal_esp32.c/.h (V2, deferred).
sim/         sim_range_sensor, sim_imu, sim_cart, sim_noise — simulated physics/noise, host-only.
estimation/  raw_speed, gravity_compensation, orientation_1d, complementary_filter — pure C99.
guidance/    v_safe, pd_controller, actuator_mapping — pure C99.
logging/     csv_logger — writes the V1 log format (docs/design.md §10).
tests/       Host-native CMake test suite (custom lightweight framework, no external deps).
docs/        design.md (spec), superpowers/plans/ (checkbox task plans).
main_host_sim.c   Host sim runner: wires hal_host + sim + estimation + guidance + logging together.
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

## Build & test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

(Commands verified current against the actual build/test suite.)

## Conventions (quick reference — full detail in docs/design.md §3)

- +X = forward, toward target. Range decreases approaching target.
  Velocity/accel positive toward target.
- gyro `[rad/s]`, 1D pitch only: `theta_k = theta_{k-1} + omega_y*dt`
  (justified by the rail constraint — no roll/yaw DOF exists to estimate).
- Main loop: 100 Hz (10 ms). ToF sensor: 20 Hz nominal. IMU: 100 Hz nominal.
- `DT_MAX_S = 2.0` — global outer sanity bound for any computed dt
  (estimator and PD-derivative alike); `dt <= 0` or `dt > DT_MAX_S` →
  reject, hold last state. Not a dropout-length limiter — see design.md §6.4.
- Two chains, never conflated: **estimation** (RAW, FUSED — speeds, scored
  vs. ground-truth velocity) vs. **guidance/control** (`control_output`,
  `servo_angle` — control outputs, scored vs. tracking of `v_safe(range)`).

## Status

V1 complete and reviewed. DEBT-1 (ESP32 HAL backend + servo output)
implemented — see `docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md`.
Builds cleanly (`idf.py build`) but has NOT been flashed/run on real
hardware in this work — that verification, plus correcting the
axis-mapping and servo placeholders against real bring-up results, is an
explicit follow-up for the project owner. DEBT-2..4 not started —
requires explicit user confirmation before beginning.
