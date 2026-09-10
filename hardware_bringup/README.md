# Hardware bring-up tests

**This is standalone bring-up firmware, not part of the V1/V2
pipeline.** Nothing in here is built by `CMakeLists.txt`, nothing in here
is imported by `hal_esp32.c`, and none of it should come to resemble the
final demo. Their only job is to produce real numbers that confirm or
correct assumptions currently baked into `docs/design.md` as informed
guesses, before those assumptions get expensive to unwind inside the
pipeline.

Each test below records results into this file (or a linked `.csv`) once
run. When a result contradicts a `docs/design.md` §5 default, that's a
signal to revisit the default in `hal/hal_host.c` / `docs/design.md` — not
a signal to change estimator/controller logic, which is designed to work
off whatever the real HAL reports, not off these specific constants.

Toolchain: **real ESP-IDF** (`imu_bringup/` is a genuine `idf.py` project,
target `esp32`), by explicit user choice over the originally-planned Arduino
framework. This means `imu_bringup/main/main.c` **does** `#include`
ESP-IDF headers (`driver/i2c_master.h`, `driver/uart.h`, etc.) — a
deliberate, scoped exception to CLAUDE.md constraint #1 for this one
standalone bring-up firmware, not a change to that constraint. It's still
outside the V1/V2 pipeline (nothing here is built by the top-level
`CMakeLists.txt` or imported by `hal_esp32.c`), and the plan is to delete
`hardware_bringup/imu_bringup/` once hardware bring-up is complete, so it
never coexists with the `tests/test_hal_boundary.sh` mechanical check that
constraint describes (that check doesn't exist yet — it's part of
DEBT-1/V2).

Tests 1-3 are all implemented in a single firmware, `imu_bringup/main/main.c`
— it auto-detects MPU6050 vs ICM20948 via `WHO_AM_I` at boot (no rebuild
needed to swap chips, just rewire and reset), then offers a UART menu
(`1`/`2`/`3`) to run each test. Test 2 streams live in a simple
`label:value,...` format readable by any serial plotter. Build/flash/monitor:

```bash
source ~/esp/esp-idf/export.sh   # once per shell
cd hardware_bringup/imu_bringup
idf.py -p /dev/ttyACM0 build flash monitor
```

## Status

| # | Test | Status | Result summary |
|---|------|--------|-----------------|
| 1 | IMU at-rest noise/bias (MPU6050 vs ICM20948) | done | both chips measured, see below |
| 2 | IMU axis/sign mapping | firmware ready, not yet run | |
| 3 | IMU achievable loop rate | done (with caveat) | ~100Hz confirmed via Test 1's cadence; tight-loop ceiling unreliable, see below |
| 4 | ToF rate + no-new-data signaling | blocked on ToF shipment | |
| 5 | ToF noise across range | blocked on ToF shipment | |
| 6 | Combined I2C bus timing (ToF+IMU[+OLED]) | blocked on ToF shipment | |

## Test 1: IMU at-rest noise/bias characterization

**Goal:** real accel/gyro noise stddev and static bias, per axis, for both
MPU6050 and ICM20948 — to replace the guessed
`HAL_HOST_IMU_ACCEL_NOISE_STDDEV = 0.05` / `HAL_HOST_IMU_GYRO_NOISE_STDDEV = 0.01`
defaults in `hal/hal_host.c` with real numbers, and to pick a chip.

**Procedure:** board stationary, level, on a solid surface. Sample
accel(x,y,z) + gyro(x,y,z) at the sensor's max practical rate for ~10s
(~1000+ samples). Print raw values over serial, capture to a `.csv`.
Repeat for both chips if both are wired up.

**Compute:** per-axis mean (bias) and stddev (noise floor), for accel and
gyro, for each chip.

**Record here:** measured stddev/bias per axis per chip; which chip you
picked and why (noise floor, library maturity, ease of wiring — whatever
actually drove the decision).

**Result (measured 2026-09-10, both chips wired simultaneously at distinct
I2C addresses, board stationary):**

MPU6050 @ 0x68:
```
Accel bias/noise (m/s^2):
  X: mean=-0.53946  stddev=0.02761
  Y: mean=-0.13525  stddev=0.02420
  Z: mean=9.31017   stddev=0.03935
Gyro bias/noise (rad/s):
  X: mean=-0.062751 stddev=0.001396
  Y: mean=0.003426  stddev=0.001325
  Z: mean=0.012793  stddev=0.001381
```

ICM20948 @ 0x69:
```
Accel bias/noise (m/s^2):
  X: mean=0.15690   stddev=0.02849
  Y: mean=0.15015   stddev=0.03016
  Z: mean=-9.96698  stddev=0.02705
Gyro bias/noise (rad/s):
  X: mean=-0.005753 stddev=0.002736
  Y: mean=-0.003689 stddev=0.002573
  Z: mean=0.001665  stddev=0.002564
```

Both chips' accel-Z magnitude land close to `SIM_GRAVITY_MPS2 = 9.81`
(MPU6050 low by ~5%, ICM20948 high by ~2%, sign flipped between the two —
the two boards are mounted in different orientations relative to each
other, which Test 2 will pin down per-chip). Gyro noise floor is
consistent with the current `HAL_HOST_IMU_GYRO_NOISE_STDDEV = 0.01`
default being conservative (real noise ~0.0013-0.0027 rad/s, well under
it); accel noise (~0.024-0.038 m/s^2) is also well under the current
`HAL_HOST_IMU_ACCEL_NOISE_STDDEV = 0.05` default. No chip-pick decision
recorded yet — both are electrically working; the choice is still open.

## Test 2: IMU axis/sign mapping

**Goal:** confirm which physical IMU axis is "up" (should read ≈+9.81
m/s² at rest, matching the `SIM_GRAVITY_MPS2` convention in
`sim/sim_imu.c`) and which gyro axis responds to hand-pitching the board
about the track's pitch axis — before this gets baked into `hal_esp32.c`'s
axis mapping into `accel_mps2[0]`(long.)/`[2]`(vert.) and `gyro_rps[1]`
(pitch rate).

**Procedure:** read raw accel+gyro continuously. Note which axis reads
≈+g at rest. Physically pitch the board by hand (e.g. prop one edge up on
a book at a known-ish angle) and note which gyro axis and which accel
axes change, and in which direction (sign).

**Record here:** which physical chip axis maps to HAL `accel_mps2[0]`
(longitudinal), `[2]` (vertical), and `gyro_rps[1]` (pitch), including
sign (does a nose-up pitch give positive or negative gyro reading on that
axis? does that match `theta_k = theta_{k-1} + omega_y*dt` with the sign
convention in `docs/design.md` §3?).

## Test 3: IMU achievable loop rate

**Goal:** confirm the 100Hz main-loop assumption (`docs/design.md` §5.6)
is achievable on this ESP32 over I2C, and how jittery it actually is.

**Procedure:** read the IMU in a tight loop with no other work, timing
each read with `micros()`. Over ~1000 iterations, compute achieved Hz and
jitter (stddev of inter-read interval).

**Record here:** achieved Hz, jitter. If far from 100Hz, note it — this
would be a real input to whether DEBT-3 (timing realism) matters sooner
than "someday."

**Result (measured 2026-09-10):** Test 1's own 1000-sample run (10ms delay
between reads — i.e. the actual real 100Hz/10ms main-loop cadence) completed
with **zero I2C errors**, both chips, every run. That's the number that
matters for `docs/design.md` §5.6 and it's confirmed with margin.

Separately, this test's own zero-delay tight-loop figure is **not
reliable** and shouldn't be used: it consistently threw `I2C software
timeout` errors (ESP-IDF's `i2c.master` log tag) across every combination
tried — 400kHz and 100kHz I2C clock, weak internal pull-ups and proper
external 2kOhm pull-ups on SDA/SCL, zero delay and a 100us inter-read
delay. None of those hardware/firmware changes cleared it, which points to
a driver-level limitation under sustained back-to-back synchronous
`i2c_master_transmit_receive` calls in this ESP-IDF version (v5.5.3) rather
than a wiring problem — real firmware never operates this way regardless
(it reads the IMU once per 10ms loop iteration, not back-to-back). Reported
numbers under this adversarial pattern were still ~1000-1100Hz even with
the errors present, i.e. 10x+ the actual 100Hz requirement, so this isn't a
blocker — just not a trustworthy "true ceiling" measurement. Not
investigated further given Test 1 already validates the number that
matters.

## Test 4: ToF rate + no-new-data signaling (blocked on ToF shipment)

**Goal:** confirm the VL53L1X can be configured for ~20Hz
(`docs/design.md` §5.1) and determine exactly how the library reports "not
ready yet" vs "fault" — this is the literal translation `hal_esp32.c`'s
`range_read()` will need from the real library's return codes to
`HAL_OK`/`HAL_NO_NEW_DATA`/`HAL_FAULT` (CLAUDE.md constraint 7: translator,
not raw cast).

**Procedure:** configure timing budget for ~50ms/measurement. Poll the
data-ready mechanism (status register or interrupt pin, whichever the
library exposes) every ~10ms in a loop; log which polls return "new data"
vs "not ready" vs an explicit fault, and the actual achieved measurement
rate.

**Record here:** achieved Hz vs the 20Hz assumption; the exact API/status
codes the library returns for each of the three cases.

## Test 5: ToF noise across range (blocked on ToF shipment)

**Goal:** confirm whether a single fixed `noise_stddev_m = 0.003`
(`hal/hal_host.c`) is a reasonable simplification across the ~1m track, or
whether real noise grows meaningfully with distance/surface.

**Procedure:** fix the sensor at several known distances along the track
(e.g. 0.1, 0.3, 0.5, 0.8, 1.0m, measured with a ruler/tape), log ~100
samples stationary at each distance.

**Record here:** stddev at each distance. Decision: keep the fixed
default, or note it as a documented simplification with the real numbers
to back that call.

## Test 6: Combined I2C bus timing (blocked on ToF shipment)

**Goal:** confirm 100Hz is achievable with ToF + IMU (+ OLED if sharing
the bus) all read back-to-back on the same loop iteration, and check for
I2C address conflicts across all three devices.

**Procedure:** wire all devices intended to share a bus, read all of them
back-to-back in a loop with no other processing, time the full loop.

**Record here:** achieved combined Hz; any address conflicts found and how
resolved (e.g., OLED moved to SPI, or a second I2C bus used).

## Optional: OLED benchtop dashboard

Not a numbered test — just a convenience sketch that prints live sensor
values to the OLED instead of (or alongside) serial, so bring-up doesn't
require a laptop tether. Not part of the V1/V2 spec; the OLED has no role
in the pipeline unless you decide later to add one.
