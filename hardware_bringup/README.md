# Hardware bring-up tests

**These are standalone Arduino-framework sketches, not part of the V1/V2
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

Toolchain: plain Arduino framework (Arduino IDE or PlatformIO), deliberately
*not* ESP-IDF — faster iteration for throwaway sketches, and keeps these
totally separate from whatever `hal_esp32.c` ends up being built with.

## Status

| # | Test | Status | Result summary |
|---|------|--------|-----------------|
| 1 | IMU at-rest noise/bias (MPU6050 vs ICM20948) | not started | |
| 2 | IMU axis/sign mapping | not started | |
| 3 | IMU achievable loop rate | not started | |
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
