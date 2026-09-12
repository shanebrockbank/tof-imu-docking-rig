# tof-imu-docking-rig — Design Doc

Status: V1 (core) design. V2 (stretch) items are listed but not designed in
detail; they are deferred until V1 is complete, tested, and reviewed with the
project owner.

## 1. Purpose

A hand-pushed cart on a ~1 m track approaches a fixed target. The cart
carries a ToF range sensor and an IMU (accel + gyro), read by an ESP32. The
project demonstrates a realistic proximity-operations signal chain —
sensing, estimation, guidance, control — built with a clean hardware
abstraction layer (HAL), fully simulatable on a host machine with zero
ESP32 hardware required, in idiomatic C99 (host logic) / C++ where the
ESP-IDF backend requires it.

This is not a flight controller and the servo has no authority over the
cart's motion (the cart is hand-pushed). It is an instrumentation and
estimation/guidance demo whose control output happens to be visualized on a
servo gauge. See §9 for what the servo represents.

## 2. Development process (binding constraints)

These are hard constraints for every change in this repo, not suggestions:

1. **HAL first, hardware last.** The full estimation/guidance/control
   pipeline must run and be tested on host before any ESP-IDF code exists.
   All logic modules (`estimation/`, `guidance/`, `common/`) are pure C99
   with zero ESP-IDF dependencies. They talk to the world only through the
   `hal.h` function-pointer struct.
2. **`hal/` vs `sim/` are different questions.** `hal/` answers "what does
   the hardware interface look like" (`hal.h`, `hal_esp32.c`, `hal_host.c` —
   thin, interface-shaped, no physics). `sim/` answers "what would the
   hardware output under these physical conditions" (`sim_range_sensor.c`,
   `sim_imu.c`, `sim_cart.c`, `sim_noise.c` — the actual simulated physics,
   noise, and cart dynamics). `hal_host.c` calls into `sim/` to produce
   readings; it contains no simulation logic itself.
3. **One file owns the hardware boundary, structurally.** `hal_esp32.c` is
   the *only* file permitted to `#include` any ESP-IDF header
   (`driver/*.h`, `esp_*.h`, `freertos/*.h`, etc.). `hal_esp32.h` re-exports
   zero ESP-IDF types — it exposes only the same plain-C interface every
   other HAL backend exposes. This is checked mechanically (see
   `tests/test_hal_boundary.sh` in the plan) by grepping every file except
   `hal_esp32.c` for ESP-IDF include patterns and failing the build if any
   match.
4. **HAL interfaces are specced before they're coded.** §5 defines units,
   rate, blocking behavior, and error/timestamp signaling for every HAL
   interface. Host test doubles model real timing and failure behavior
   (e.g. "no new sample yet" vs "sensor fault"), not just plausible values.
5. **Framing: "host-simulatable, HIL-ready structure" — not "HIL-ready."**
   V1 does not include a hardware-in-the-loop test fixture. It is
   *structured so one could be added later without touching
   estimation/guidance/control code*, because that code only ever sees the
   HAL interface, never a concrete backend.
6. **ISR boundary: polled, by deliberate choice, in V1.** Neither the host
   sim nor the (deferred) ESP32 backend uses a data-ready interrupt in V1.
   Both `IRangeSensor` and `IImu` are polled from the main loop at the main
   loop's tick rate. This is a deliberate simplification, not an oversight
   — justified in §5.5. If a future ESP32 backend adopts interrupt-driven
   reads (e.g. IMU data-ready IRQ), the required boundary is: ISR sets a
   flag/pushes to a queue only; the main loop polls that flag/queue and
   calls into HAL from ordinary task context. Estimation/guidance/control
   code must never be reachable from ISR context. This is documented here
   so it's decided *before* anyone is tempted to call a filter update from
   an interrupt handler.
7. **Numeric regression gate.** The host-native test suite is run after
   every implementation step, and the pass count (e.g. "42/42 tests
   passing") is reported. A step is not complete until the gate passes.
8. **Translation shims, never raw casts.** Any two modules with
   enums/types representing the same concept but not guaranteed to share
   underlying values (e.g. a HAL error code vs. a sim fault code) get an
   explicit translator function.
9. **Known debt is tracked with IDs**, not silently skipped. See §11.
10. **`CLAUDE.md` scopes context** — hardware reference material (pinouts,
    datasheets, servo timing specs) is not opened unless the current task
    specifically concerns `hal_esp32.c`. None of that material exists yet
    in V1 since the ESP32 backend is a V2 item.
11. **Fixed-size state, no dynamic allocation after init.** Every
    filter/controller/logger has a fixed-size struct. One `.c`/`.h` pair
    per logical module.
12. **Spec → plan → execute.** This doc is the spec. A checkbox task plan
    is derived from it. Implementation proceeds task by task, gate-checked
    after each task.

## 3. Coordinate / sign / unit conventions

- **+X** = cart's forward direction, toward the target.
- **range** `[m]`: distance from sensor to target; decreases while
  approaching.
- **velocity** `[m/s]`: positive toward the target (i.e. positive when
  range is decreasing).
- **longitudinal acceleration** `[m/s²]`: positive toward the target.
- **gyro** `[rad/s]`, about the pitch (Y) axis only — see §3.1.
- **accel**: raw 3-axis `[m/s²]` as reported by the IMU HAL (includes
  gravity, per standard accelerometer convention); gravity is removed by
  the named gravity-compensation module (§6.2), not inline in the fusion
  code.
- **theta** `[rad]`: 1D pitch angle, integrated as
  `theta_k = theta_{k-1} + omega_y * dt`.
- **time** `[s]`, monotonic, `double` or `uint64_t` microseconds at the HAL
  boundary (see §5.4) converted to `double` seconds for all logic-layer
  math.
- All measurements carry a timestamp. The estimator is **timestamp-driven**
  — it computes `dt` from consecutive timestamps and validates it (§6.4);
  it never assumes a fixed tick period.

### 3.1 Why 1D pitch only, not full 3D attitude

The cart is physically constrained to a single rail: it can only translate
along the track's long axis and pitch slightly about that axis's
perpendicular (e.g. due to wheel/track compliance or a bump), which is
exactly the rotation that couples into the longitudinal-acceleration
measurement via gravity. It has no mechanism for roll (no lateral tilt
freedom — the rail fixes that DOF) and no mechanism for yaw (the rail fixes
heading; the cart cannot rotate about vertical without leaving the track).
A full 3D quaternion/rotation-matrix attitude estimator would be strictly
more general than the hardware can produce evidence for: there is no sensor
information and no physical degree of freedom to distinguish "yaw drifted"
or "rolled" states, so a 3D filter would either silently rely on
unobservable states or need additional constraints to fake observability —
complexity with no corresponding physical payoff. A single scalar pitch
integrated from `omega_y` is both fully sufficient (it is the only rotation
that affects the longitudinal-acceleration reading through gravity) and
fully justified by the rail constraint. This is a deliberate scope
decision, not a deferred one — 3D attitude is not on the V2 list.

## 4. Two separate chains

**Estimation chain** produces *speed estimates* (RAW, FUSED). **Guidance /
control chain** consumes FUSED and produces an *actuator command*
(`control_output`, `servo_angle`). These are categorically different
quantities and must never be conflated:

- `control_output` / `servo_angle` (TUNED) must never be described as "a
  better speed estimate" anywhere in code, tests, or docs. It is a control
  output derived from FUSED speed *and* a guidance error term
  (`speed_error = FUSED − v_safe(range)`), not a refinement of FUSED.
- The two chains are evaluated with separate metrics (§8) and are never
  compared against each other directly, because they don't estimate the
  same thing: the estimation chain is scored against simulated
  ground-truth *velocity*; the control chain is scored against *tracking
  of `v_safe(range)`*, a guidance target, not a physical quantity.

```
                 ┌─────────────────────── ESTIMATION CHAIN ───────────────────────┐
ToF range ──────►│ backward diff ──────────────────────────────────► RAW speed    │
                  │                                                                │
IMU accel ───────►│ gravity compensation ──► a_longitudinal ──┐                    │
IMU gyro  ───────►│ theta integration (1D pitch) ─────────────┘                   │
                  │                     │                                          │
                  │                     ▼                                          │
range + accel/gyro│         complementary filter (v_pred, ToF-corrected)          │
                  │                     │                                          │
                  └─────────────────────┼──────────────────────────► FUSED speed  ┘
                                         │
                 ┌───────────────────── GUIDANCE / CONTROL CHAIN ───────────────────┐
range ──────────►│ v_safe(range) ──┐                                                │
                  │                 ▼                                               │
FUSED speed ─────►│ speed_error = FUSED − v_safe(range) ──► PD controller ──►        │
                  │                              (D-term filtered, output saturated) │
                  │                                          control_output [-1,1]   │
                  │                                                    │             │
                  │                                                    ▼             │
                  │                                     actuator_mapping.c           │
                  │                                                    │             │
                  └────────────────────────────────────────────────────┼─────────────┘
                                                                        ▼
                                                                  servo_angle [deg]
```

## 5. HAL interfaces

All HAL interfaces are exposed through a single function-pointer struct
(`hal_t`) defined in `hal/hal.h`, populated once at startup by either
`hal_host_create()` or `hal_esp32_create()`. Estimation/guidance/control
code only ever sees `hal_t`.

Common types (`common/types.h`):

```c
typedef struct { double t_s; } timestamp_t;   /* seconds, monotonic, host-domain double */

typedef enum {
    HAL_OK = 0,
    HAL_NO_NEW_DATA,   /* polled and nothing new since last read; not an error */
    HAL_FAULT,         /* sensor reported/simulated a fault (e.g. out of range, I2C nack) */
} hal_status_t;
```

`HAL_NO_NEW_DATA` is a first-class outcome, not an error — see §5.5 (polled
model). Callers must handle it distinctly from `HAL_FAULT`.

### 5.1 `IRangeSensor`

Represents the ToF sensor (e.g. VL53L1X-class).

```c
typedef struct {
    hal_status_t (*read)(void *ctx, double *range_m, timestamp_t *ts);
} range_sensor_if_t;
```

- **Units:** `range_m` in meters, positive, decreasing toward the target.
- **Rate:** sensor produces a new reading at a nominal 20 Hz (50 ms
  period). This is the sensor's own conversion rate, independent of the
  caller's poll rate.
- **Blocking behavior:** `read()` is **non-blocking**. It is polled once
  per main-loop tick (100 Hz, §5.6). If no new sample has completed since
  the last call, it returns `HAL_NO_NEW_DATA` and leaves `*range_m`/`*ts`
  unwritten. If a new sample is ready, it returns `HAL_OK` and writes the
  value and the timestamp *the sample was taken at* (not the poll time).
  On a simulated/real sensor fault (out-of-range return, timeout), it
  returns `HAL_FAULT` and leaves outputs unwritten.
- **Timestamp signaling:** `ts` is the acquisition time of the range
  sample, in the same monotonic clock domain as `IClock` (§5.4). This is
  what makes backward-difference dt correct even though the sensor is
  polled faster than it updates.
- **Error signaling:** `HAL_NO_NEW_DATA` (normal, expected most ticks at
  100 Hz poll / 20 Hz sensor) vs `HAL_FAULT` (sensor problem — estimator
  must treat this like a dropout, same as a stretch of `HAL_NO_NEW_DATA`).

### 5.2 `IImu`

```c
typedef struct {
    hal_status_t (*read)(void *ctx, double accel_mps2[3], double gyro_rps[3], timestamp_t *ts);
} imu_if_t;
```

- **Units:** `accel_mps2` in m/s² (includes gravity, standard accelerometer
  convention, body frame X/Y/Z); `gyro_rps` in rad/s (body frame X/Y/Z).
  Only `accel[0]` (X, longitudinal), `accel[2]` (Z, vertical) and
  `gyro[1]` (Y, pitch rate) are used by V1 logic (§3.1) — X/Z/Y indices
  are fixed by this struct's ordering, not inferred.
- **Rate:** sampled every main-loop tick, 100 Hz (10 ms). Unlike the ToF
  sensor, the IMU is fast enough that "new data every poll" is the
  expected case, but the interface still reports `HAL_NO_NEW_DATA` if a
  backend genuinely has nothing new (e.g. a real IMU FIFO underrun),
  because the estimator's dt validation (§6.4) must not assume every poll
  yields fresh IMU data.
- **Blocking behavior:** non-blocking, identical polling contract to
  `IRangeSensor`.
- **Timestamp signaling:** acquisition timestamp of the IMU sample, same
  clock domain as `IClock`.
- **Error signaling:** `HAL_NO_NEW_DATA` / `HAL_FAULT` as above.

### 5.3 `IActuator`

```c
typedef struct {
    hal_status_t (*set_angle_deg)(void *ctx, double angle_deg);
} actuator_if_t;
```

- **Units:** degrees, absolute servo angle (not a delta).
- **Rate:** called once per main-loop tick, 100 Hz. No internal rate
  limiting at the HAL level — rate limiting, if any, is a guidance/control
  concern (`actuator_mapping.c`), not a HAL concern, and is out of scope
  for V1 (basic saturation only; rate limiting is V2, ID `DEBT-4`).
- **Blocking behavior:** non-blocking, "fire and forget" — real servo PWM
  backends do not block waiting for the servo to physically reach the
  angle. Returns `HAL_OK` on successful command dispatch, `HAL_FAULT` if
  the backend rejects the value (e.g. out of the actuator's mechanical
  range, checked *inside* the HAL backend, not by the caller).
- **No timestamp:** commands are fire-and-forget; the caller's own
  `IClock` reading for that tick is the authoritative time if a log needs
  one.

### 5.4 `IClock`

```c
typedef struct {
    timestamp_t (*now)(void *ctx);
} clock_if_t;
```

- **Units:** seconds, monotonic, `double`, arbitrary epoch (not
  wall-clock). On host, backed by a simulated clock that advances exactly
  one main-loop tick (10 ms nominal) per call to the sim step function —
  deterministic, not wall-clock-driven, so host tests are reproducible.
  On ESP32 (V2), backed by `esp_timer_get_time()` converted to seconds.
- **Rate/blocking:** called at least once per main-loop tick; cheap,
  non-blocking, no side effects beyond reading.

### 5.5 Why polled, not interrupt-driven, in V1

Both sensors update far slower (IMU: 100 Hz) or much slower (ToF: 20 Hz)
than would justify interrupt latency savings for this application — the
control loop has no sub-millisecond timing requirement (it's driving a
gauge servo, not a flight-critical actuator). Polling at a fixed 100 Hz
main-loop rate keeps the timing model simple and identical between host
sim and (future) hardware, and keeps 100% of estimation/guidance/control
code free of any concurrency concerns. This is stated as a deliberate
choice per working-agreement item 6, not a gap: if a future revision needs
interrupt-driven IMU reads for latency reasons, the required boundary
(ISR → flag/queue → poll) is already documented in §2 item 6 so it can be
added without touching estimation/guidance/control code.

### 5.6 Main loop rate

The main loop (host sim runner and, later, the ESP32 `app_main`) ticks at
**100 Hz** (10 ms nominal period). This is the rate at which `IClock`
advances, `IImu` is polled, `IRangeSensor` is polled (returning
`HAL_NO_NEW_DATA` on ~4 out of 5 ticks, since the sensor itself updates at
20 Hz), and `IActuator` is commanded.

## 6. Estimation chain design

### 6.1 RAW speed — mechanics

RAW speed is computed **only** on ticks where `IRangeSensor.read()` returns
`HAL_OK` (a genuinely new sample), using backward finite differencing:

```
v_raw = (r_k - r_{k-1}) / (t_k - t_{k-1})
```

subject to dt validation (§6.4). On every other tick (`HAL_NO_NEW_DATA` or
`HAL_FAULT`, or a rejected dt), RAW speed is **held** at its last computed
value and flagged **stale**:

```c
typedef struct {
    double value_mps;
    bool   is_stale;   /* true = held from a previous tick, not recomputed this tick */
} raw_speed_t;
```

RAW is never recomputed against an unchanged range value and never divided
by a near-zero/invalid dt. The `is_stale` flag is carried into the CSV log
(§10) so held-vs-fresh values are distinguishable after the fact.

### 6.2 Gravity compensation (`estimation/gravity_compensation.c`)

Own named module, not inlined into the filter:

```
a_longitudinal = a_x * cos(theta) - a_z * sin(theta)
```

using the 1D pitch `theta` from §6.3. Input: raw `accel_mps2[3]` and
current `theta`. Output: scalar `a_longitudinal` in m/s², gravity removed
to the extent the pitch estimate captures the tilt.

### 6.3 Orientation (1D pitch only)

```
theta_k = theta_{k-1} + omega_y * dt
```

integrated every IMU tick, subject to dt validation (§6.4). See §3.1 for
why this is 1D and not 3D. No bias estimation/correction on `theta` in V1
(theta feeds only gravity compensation, and any residual bias there is a
second-order effect on an already-secondary term); this is not on the
known-debt list because it's judged out of scope for what this instrument
needs to demonstrate, not deferred functionality.

### 6.4 dt validation (applies everywhere a dt is computed)

Before using any computed `dt` — ToF backward differencing, IMU/theta
integration, complementary-filter propagation, PD derivative term — the
consumer validates:

```c
#define DT_MAX_S 2.0   /* generous outer sanity bound, not a dropout-length limit */

bool dt_is_valid(double dt) {
    return dt > 0.0 && dt <= DT_MAX_S;
}
```

If `dt <= 0` (duplicate/out-of-order timestamp) or `dt > DT_MAX_S`
(clock glitch, simulated clock quantization artifact, or similarly absurd
value), the step is **rejected**: the consumer falls back to zero
propagation (hold last state) rather than dividing by, or integrating
over, a bad dt. `DT_MAX_S = 2.0 s` is chosen as ~40x the nominal ToF
sample period (50 ms) / ~200x the nominal main-loop period (10 ms) — large
enough that a real (if long) ToF dropout is still a *valid* dt and gets a
real (if smoothed-over-a-longer-window) backward-difference speed, but
small enough to catch genuine glitches (e.g. a timestamp overflow/reset
artifact). Dropout recovery (§8) is exercised with gaps well under
`DT_MAX_S` (hundreds of ms) specifically so the test distinguishes "long
gap, still valid" from "pathological, rejected."

### 6.5 Complementary filter (`estimation/complementary_filter.c`)

V1 uses a complementary filter. **A Kalman filter is an explicit, named
V2/future-extension possibility (`DEBT-2`) and is not implemented in V1.**

```
v_pred = v_est + a_longitudinal * dt              (every IMU tick, dt-validated)
on ToF update (HAL_OK this tick, dt-validated against ToF's own last ts):
    v_est = alpha * v_pred + (1 - alpha) * v_tof
otherwise:
    v_est = v_pred
```

`v_tof` is the freshly computed RAW speed for *this* tick (not the held/
stale value — the filter only pulls in a ToF correction on ticks where RAW
was freshly computed, i.e. `is_stale == false`). Default `alpha = 0.90`
(90% weight on the IMU-propagated prediction, 10% pulled toward the fresh
ToF-derived speed per correction) — chosen so ToF, not IMU integration,
dominates the long-term answer (§ README) while IMU still visibly smooths
between updates; tunable during testing.

**The IMU provides only short-term velocity propagation between valid ToF
updates. The estimator does not rely on IMU integration for long-term
position/velocity accuracy** — every ToF update pulls the estimate back
toward ground truth, bounding IMU-bias-driven drift. This must appear
verbatim in the module's header comment, not just this doc.

## 7. Guidance / control chain design

### 7.1 `v_safe(range)` (`guidance/v_safe.c`)

```
v_safe(r) = clamp(k * sqrt(r), v_floor, v_cap)
```

Defaults for a ~1 m track: `k = 0.5`, `v_cap = 0.6 m/s` (caps the allowed
speed near the start of the approach, where `sqrt(r)` would otherwise keep
growing as `r` grows past the track length), `v_floor = 0.03 m/s` (floored
near contact so the target speed approaches a small positive crawl rather
than exactly zero, avoiding a divide/derivative singularity right at
dock). This is the classic glideslope/range-gated speed-limit shape used
in real proximity-ops guidance (see README) — allow closing speed
proportional to remaining range budget, capped at a sane maximum and
floored above zero near contact.

### 7.2 PD controller (`guidance/pd_controller.c`)

```
speed_error = FUSED - v_safe(range)
control_output_raw = Kp * speed_error + Kd * d(speed_error)/dt
```

- The PD controller **never** knows about servo angles/degrees — its
  output is a dimensionless-normalized `control_output`, clamped to
  `[-1, +1]` (saturation). Mapping to a physical unit is
  `actuator_mapping.c`'s job only.
- The derivative term uses the same dt validation as §6.4; on an invalid
  dt, the derivative term contributes 0 for that tick (fall back to
  proportional-only) rather than dividing by a bad dt.
- **D-term noise amplification is expected and is itself a demonstrable
  point** (§8): differentiating a noisy `speed_error` amplifies
  high-frequency noise. V1 applies a first-order low-pass (EMA) filter to
  the derivative term only (not to `speed_error` itself, so the P-term
  stays responsive):

```
d_filtered_k = (dt / (tau + dt)) * d_raw_k + (tau / (tau + dt)) * d_filtered_{k-1}
```

  default `tau = 0.05 s`. The controller is able to report *both*
  `control_output` computed with the unfiltered D-term and with the
  filtered D-term on the same run, specifically so the comparison in §8
  can be shown side by side without re-running the sim.
- Default gains `Kp = 1.5`, `Kd = 0.1` — tunable during testing; not
  claimed optimal, just a stable, demonstrable starting point.
- Output saturation: `control_output = clamp(control_output_raw, -1, +1)`,
  applied after the filtered-D computation.

### 7.3 Actuator mapping (`guidance/actuator_mapping.c`)

```
servo_angle_deg = SERVO_CENTER_DEG + control_output * SERVO_HALF_RANGE_DEG
```

Defaults: `SERVO_CENTER_DEG = 90`, `SERVO_HALF_RANGE_DEG = 45` (range
45–135°). This is a distinct module specifically so the entire
guidance/control stack is testable and meaningful with zero hardware and
zero knowledge of servo geometry. Per working-agreement item, the module's
header carries this one-line comment verbatim:

```c
/* This servo acts as a visual glideslope-error gauge, not a physical
   actuator — the cart is hand-pushed and the servo has no authority over
   its motion. */
```

## 8. Measurement / evaluation contract

Simulated ground truth (`sim_cart`'s true position/velocity) is carried
internally through every test/sim run for validation only, and is **never**
fed into the estimator or controller — it is wired straight from
`sim_cart` into the metrics/logging layer, bypassing `hal_host.c` entirely,
so there is no code path by which estimation logic could accidentally read
it.

**Estimation chain** (RAW and FUSED, reported separately):
- RMSE vs. simulated ground-truth velocity
- mean bias
- variance
- dropout recovery: time to reconverge (within a defined tolerance band of
  ground truth) after a ToF gap of specified duration
- response lag after a step change in true velocity

**Control chain**:
- RMS speed-tracking error relative to `v_safe(range)`
- `control_output` variance
- `control_output` derivative/jitter metric, computed **twice** on the
  same run — once from the unfiltered D-term output, once from the
  filtered — to show the amplification and the fix side by side
- actuator saturation percentage (fraction of ticks at ±1 clamp)
- response time to a step change in `speed_error`

Estimation and control chains are evaluated with separate metric sets and
are never compared directly to each other (§4) — RAW/FUSED are speeds,
scored against ground-truth velocity; `control_output`/`servo_angle` are
control outputs, scored against tracking of a guidance target.

## 9. What the servo represents

The servo is a **visual glideslope-error gauge** for this demo. The cart is
hand-pushed; the servo has no authority over the cart's motion. Its angle
visualizes how far `FUSED` speed is from the guidance target `v_safe(range)`
at the current range, exactly as a cockpit glideslope indicator shows
deviation from a target path without itself flying the aircraft.

## 10. Logging (V1)

One CSV row per main-loop tick, columns:

```
t_s, true_range_m, true_velocity_mps, raw_speed_mps, raw_is_stale,
fused_speed_mps, v_safe_mps, speed_error_mps, control_output_unfiltered,
control_output_filtered, servo_angle_deg
```

`true_range_m`/`true_velocity_mps` come directly from `sim_cart` (ground
truth, logged for validation, never fed to the estimator per §8).

**Hardware variant:** the ESP32 backend (`main_esp32.c`, DEBT-1) has no
simulated ground truth, so it uses `csv_logger_write_row_hw()` instead —
identical columns minus `true_range_m`/`true_velocity_mps` (no hardware
analog), with `measured_range_m` (the real last-known ToF reading) in
`true_range_m`'s old column position.

## 11. Known debt (V2 candidates, tracked explicitly)

| ID | Item | Notes |
|----|------|-------|
| DEBT-1 | ESP32 hardware backend (`hal_esp32.c`) + real servo output | The actual physical build. Requires §2 item 3 boundary to already hold. |
| DEBT-2 | Kalman filter as alternate estimator, compared side-by-side against the V1 complementary filter using the same §8 contract | Explicitly not built in V1. |
| DEBT-3 | Richer timing realism: injected ToF latency, IMU timing jitter, out-of-order/stale measurement handling, latency metrics under these conditions | V1's dt validation (§6.4) handles pathological dt but does not inject or specifically characterize jitter/latency. |
| DEBT-4 | Actuator deadband + rate-limiting beyond basic saturation | V1 has saturation only (§7.2). |

## 12. V1 vs. V2 scope

**V1 (this plan):** HAL interfaces + host backend, sim modules, estimation
chain (RAW + gravity comp + 1D pitch + complementary filter + dt
validation), guidance/control chain (v_safe + PD w/ filtered D-term +
actuator mapping), CSV logging, host-native CMake test suite covering the
full §8 contract plus staleness/dt-validation edge cases, README.

**V2 (stretch, not started without explicit confirmation):** DEBT-1
through DEBT-4 above.
