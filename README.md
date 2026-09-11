# tof-imu-docking-rig

A hand-pushed cart on a ~1 m rail approaches a fixed target. The cart
carries a Time-of-Flight (ToF) range sensor and an IMU (accelerometer +
gyro), read by an ESP32. The project is a realistic proximity-operations
signal chain — **sensing → estimation → guidance → control** — built
HAL-first, so the entire pipeline runs and is tested on a host machine
with **zero ESP32 hardware required**.

This is not a flight controller and the servo has no authority over the
cart's motion — the cart is hand-pushed. It is an instrumentation and
estimation/guidance demo whose control output happens to be visualized on
a servo gauge (see "What the servo represents" below).

The full binding spec is `docs/design.md`; this file is meant to stand on
its own for a reader who never opens it.

## Contents

- [Why ToF dominates long-term velocity accuracy](#why-tof-dominates-long-term-velocity-accuracy)
- [Why the IMU helps between ToF updates — and only between updates](#why-the-imu-helps-between-tof-updates--and-only-between-updates)
- [Why differentiating ToF amplifies noise, and how staleness is handled](#why-differentiating-tof-amplifies-noise-and-how-staleness-is-handled)
- [Why orientation is 1D pitch only, not full 3D attitude](#why-orientation-is-1d-pitch-only-not-full-3d-attitude)
- [Why the PD derivative term amplifies noise, and how that's addressed](#why-the-pd-derivative-term-amplifies-noise-and-how-thats-addressed)
- [Why the estimator is timestamp-driven, not fixed-dt](#why-the-estimator-is-timestamp-driven-not-fixed-dt)
- [Why the hardware boundary lives in one file](#why-the-hardware-boundary-lives-in-one-file)
- [Why estimation and control are scored separately, never against each other](#why-estimation-and-control-are-scored-separately-never-against-each-other)
- [What the servo physically represents](#what-the-servo-physically-represents)
- [Coordinate / sign / unit conventions](#coordinate--sign--unit-conventions)
- [HAL/sim separation and development process](#halsim-separation-and-development-process)
- [`v_safe(range)`: shape and the glideslope analogy](#v_saferange-shape-and-the-glideslope-analogy)
- [V1 measurement results](#v1-measurement-results)
- [Build & test](#build--test)
- [Known debt (V2 candidates)](#known-debt-v2-candidates)

## Why ToF dominates long-term velocity accuracy

The ToF sensor measures absolute range directly against the real world.
Any velocity derived from it (via backward differencing, see below) is
noisy on a single sample-to-sample basis, but it cannot drift — every new
range reading is an independent, fresh measurement of ground truth, not
an accumulation of past errors.

IMU integration is the opposite: integrating acceleration to get velocity
accumulates every small bias and noise sample forever. A constant
accelerometer bias of even a few mm/s² integrates into an unbounded
velocity error over time if nothing ever corrects it.

So the fusion in this pipeline is deliberately built to let ToF, not IMU
integration, own the long-term answer: every valid ToF update pulls the
fused estimate back toward a fresh, independent ground-truth-referenced
measurement, which caps how far IMU-driven drift can run before it gets
corrected. The "IMU bias drift stays bounded" test below demonstrates this
directly — a persistent 0.3 m/s² accelerometer bias (huge, by real IMU
standards) still produces a bounded fused-speed RMSE, not a runaway one,
because ToF corrections keep resetting the error.

## Why the IMU helps between ToF updates — and only between updates

The ToF sensor updates at a nominal 20 Hz; the main loop runs at 100 Hz.
Between ToF samples there would otherwise be nothing but a held, stale
speed value. The IMU is sampled every tick (100 Hz), so it is used to
*propagate* the velocity estimate forward smoothly in those gaps
(`v_pred = v_est + a_longitudinal * dt`), giving a much more useful
between-updates signal than "just hold the last ToF-derived value."

But the IMU's role is deliberately kept short-term only. It is never
allowed to be the thing the long-term answer depends on — every fresh ToF
sample pulls the estimate back down to 10% weight toward the raw
ToF-derived speed (`alpha = 0.90` favors the IMU-propagated prediction at
each individual correction, but corrections happen often enough, and
reliably enough, that the *net* long-run behavior is ToF-anchored, not
IMU-anchored). This split — "smooth the gaps, but never own the answer" —
is what a complementary filter is *for*, and is why this project doesn't
reach for a full state estimator like a Kalman filter for V1 (that's
`DEBT-2`, a deliberate V2 candidate, not an oversight).

## Why differentiating ToF amplifies noise, and how staleness is handled

RAW speed is computed by backward finite-differencing consecutive range
samples: `v_raw = (r_k - r_{k-1}) / (t_k - t_{k-1})`. Differentiation is a
classic noise amplifier — any measurement noise on `r_k`/`r_{k-1}` gets
divided by a (typically small) `dt`, so small range jitter turns into
comparatively large speed jitter. This is visible directly in the
measurement results below: RAW's RMSE against ground-truth velocity is
roughly 10x FUSED's.

RAW is **only ever recomputed on a tick where the ToF sensor actually
returned a genuinely new sample** (`HAL_OK`). On every other tick — no new
sample yet (`HAL_NO_NEW_DATA`, the normal case ~4 out of 5 ticks at a
100 Hz loop / 20 Hz sensor) or a sensor fault (`HAL_FAULT`) — RAW speed is
**held** at its last computed value and flagged stale:

```c
typedef struct {
    double value_mps;
    bool   is_stale;   /* true = held from a previous tick, not recomputed this tick */
} raw_speed_t;
```

RAW is never recomputed against an unchanged range value and never divided
by a near-zero or invalid `dt`. The complementary filter only pulls a ToF
correction into FUSED on ticks where RAW is fresh (`is_stale == false`) —
a stale RAW value never gets treated as if it were a new measurement. The
`is_stale` flag is also carried straight into the CSV log column
`raw_is_stale`, so held-vs-fresh values are distinguishable after the
fact.

## Why orientation is 1D pitch only, not full 3D attitude

The cart is physically constrained to a single rail: it can only
translate along the track's long axis, and pitch slightly about the axis
perpendicular to that (e.g. from wheel/track compliance or a bump) — which
is exactly the rotation that couples into the longitudinal-acceleration
reading through gravity. It has no roll freedom (the rail fixes lateral
tilt) and no yaw freedom (the rail fixes heading; the cart can't rotate
about vertical without leaving the track).

A full 3D quaternion/rotation-matrix attitude estimator would be strictly
more general than the hardware can produce evidence for — there is no
sensor information and no physical degree of freedom to tell "yaw
drifted" apart from "didn't," so a 3D filter would either silently rely
on states nothing observes, or need fake constraints to pretend
observability. A single scalar pitch, integrated from the gyro's Y-axis
rate (`theta_k = theta_{k-1} + omega_y * dt`), is both sufficient (it's
the only rotation that affects the longitudinal-acceleration reading
through gravity) and fully justified by the rail constraint. This is a
deliberate scope decision, not deferred work — 3D attitude never appears
on the V2/debt list.

## Why the PD derivative term amplifies noise, and how that's addressed

The guidance/control chain computes `speed_error = FUSED - v_safe(range)`
and feeds it to a PD controller:

```
control_output_raw = Kp * speed_error + Kd * d(speed_error)/dt
```

Just like the RAW-speed differencing above, the D-term differentiates a
noisy signal (`speed_error`, which inherits FUSED's residual noise) —
any high-frequency jitter in `speed_error` gets amplified by the
derivative. Left unfiltered, this shows up directly as a jumpy,
high-variance `control_output`, and therefore a jumpy servo gauge.

V1 fixes this by applying a first-order low-pass (EMA) filter to the
*derivative term only* — not to `speed_error` itself, so the proportional
term stays fully responsive:

```
d_filtered_k = (dt / (tau + dt)) * d_raw_k + (tau / (tau + dt)) * d_filtered_{k-1}
```

with `tau = 0.05 s` by default. The controller reports `control_output`
computed both ways (`control_output_unfiltered` and
`control_output_filtered`) from the *same* run, specifically so the
before/after comparison doesn't require re-running the sim. The measured
effect (see "V1 measurement results" below): on a deliberately noisy run
with a raised `Kd`, unfiltered jitter RMS was **0.2497** against filtered
jitter RMS of **0.0409** — roughly a 6x reduction from the EMA filter
alone, a clear demonstration of both the problem and the fix on the same
data.

## Why the estimator is timestamp-driven, not fixed-dt

Every measurement carries its own acquisition timestamp rather than the
estimator assuming a fixed tick period. This matters because the ToF
sensor's own conversion rate (20 Hz) is independent of, and slower than,
the main loop's poll rate (100 Hz) — most polls return "nothing new yet,"
and computing `dt` from real timestamps (rather than assuming every tick
represents a fresh 10 ms of new data) is what makes the backward-
difference RAW speed correct even though the sensor is polled far more
often than it updates.

Before any computed `dt` is used anywhere in the pipeline (ToF backward
differencing, gyro/theta integration, complementary-filter propagation,
the PD derivative term), it's validated:

```c
#define DT_MAX_S 2.0   /* generous outer sanity bound, not a dropout-length limit */
bool dt_is_valid(double dt) { return dt > 0.0 && dt <= DT_MAX_S; }
```

If `dt <= 0` (a duplicate or out-of-order timestamp) or `dt > DT_MAX_S`
(a clock glitch or similarly absurd value), the step is **rejected**: the
consumer falls back to holding its last state (zero propagation) instead
of dividing by, or integrating over, a bad `dt`. `DT_MAX_S = 2.0 s` is
about 40x the nominal ToF sample period (50 ms) and about 200x the main
loop period (10 ms) — large enough that a real, if long, ToF dropout is
still treated as a *valid* dt (and gets a real, if smoothed-over-a-longer-
window, backward-difference speed), but small enough to catch genuine
timestamp glitches. This is why the dropout-recovery test below uses a
~300 ms gap (well under 2.0 s, so it's a "long but valid" dropout, not a
rejected one) and still shows the estimator reconverging.

## Why the hardware boundary lives in one file

`hal/hal_esp32.c` is the *only* file in this repo permitted to
`#include` any ESP-IDF header (`driver/*.h`, `esp_*.h`, `freertos/*.h`,
`sdkconfig.h`, etc.). `hal/hal_esp32.h` re-exports zero ESP-IDF types —
its declared interface is identical in shape to `hal_host.h`, the
simulated backend. Every module above the HAL (`estimation/`,
`guidance/`, `common/`) talks to the world *only* through the `hal_t`
function-pointer struct — never through a concrete backend — so none of
that logic can ever accidentally acquire an ESP-IDF dependency, and the
exact same estimation/guidance code runs unmodified against the
simulated backend today and the real ESP32 backend once `DEBT-1` lands.

This is checked mechanically, not just by convention:
`tests/test_hal_boundary.sh` greps every file in the repo *except*
`hal_esp32.c` for ESP-IDF include patterns, and fails the build if any
match. It runs as its own ctest entry (`test_hal_boundary`) alongside
every C test suite.

## Why estimation and control are scored separately, never against each other

The estimation chain produces *speed estimates* — RAW and FUSED — scored
against simulated ground-truth **velocity**. The guidance/control chain
produces an *actuator command* — `control_output` and `servo_angle` —
scored against **tracking of `v_safe(range)`**, a guidance target, not a
physical quantity. These are categorically different things being
measured against categorically different references, so they're never
compared to each other directly, and `control_output`/`servo_angle` is
never described as "a better speed estimate" anywhere in this codebase —
it's a control output derived from FUSED speed *and* a guidance error
term (`speed_error = FUSED - v_safe(range)`), not a refinement of FUSED.

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

Simulated ground truth (`sim_cart`'s true position/velocity) is threaded
through every test/sim run for *validation only*, wired straight from
`sim_cart` into the metrics/logging layer, bypassing the HAL entirely —
there is no code path by which estimation or control logic could
accidentally read it.

## What the servo physically represents

Quoted verbatim from `guidance/actuator_mapping.h`:

```c
/* This servo acts as a visual glideslope-error gauge, not a physical
   actuator — the cart is hand-pushed and the servo has no authority over
   its motion (docs/design.md §7.3, §9). */
```

The servo's angle visualizes how far FUSED speed currently is from the
guidance target `v_safe(range)` at the current range. It commands nothing
about the cart — the cart is hand-pushed and there is no feedback path
from the servo back into the plant.

## Coordinate / sign / unit conventions

| Quantity | Unit | Sign convention |
|---|---|---|
| `+X` axis | — | forward, toward the target |
| range | m | distance sensor→target; **decreases** approaching the target |
| velocity | m/s | positive **toward** the target (positive when range is decreasing) |
| longitudinal acceleration | m/s² | positive toward the target |
| gyro (`omega_y`) | rad/s | pitch axis only (see 1D-pitch rationale above) |
| accel (`accel_mps2[3]`) | m/s² | raw 3-axis, body frame, **includes gravity** (standard accelerometer convention); gravity is removed by the named gravity-compensation module, not inline in the fusion code |
| theta (pitch) | rad | `theta_k = theta_{k-1} + omega_y * dt` |
| time / timestamps | s | monotonic `double`, arbitrary epoch; `uint64_t` microseconds only at the raw HAL boundary |
| main loop rate | — | 100 Hz (10 ms nominal tick) |
| ToF sensor rate | — | 20 Hz nominal (independent of the 100 Hz poll rate) |
| IMU rate | — | 100 Hz nominal (sampled every tick) |
| `DT_MAX_S` | s | 2.0 — global outer sanity bound for any computed `dt` |

## HAL/sim separation and development process

Two questions are kept deliberately separate:

- **`hal/`** answers "what does the hardware interface look like" —
  `hal.h` (the function-pointer interface structs), `hal_host.c`
  (simulation-backed implementation), `hal_esp32.c` (V2, deferred). Thin,
  interface-shaped, **no physics**.
- **`sim/`** answers "what would the hardware output under these physical
  conditions" — `sim_range_sensor`, `sim_imu`, `sim_cart`, `sim_noise`:
  the actual simulated physics, noise, and cart dynamics. `hal_host.c`
  *calls into* `sim/` to produce readings; it contains no simulation math
  of its own.

Other binding process agreements (full list in `CLAUDE.md`/
`docs/design.md` §2):

- **HAL first, hardware last.** The full pipeline runs and is tested on
  host before any ESP-IDF code exists. `estimation/`, `guidance/`, and
  `common/` are pure C99 with zero hardware dependencies of any kind —
  they only ever see `hal_t`.
- **Polled, not interrupt-driven, in V1.** Both sensors are polled from
  the main loop at the main loop's tick rate; neither the host sim nor
  the (deferred) ESP32 backend uses a data-ready interrupt. If a future
  backend needs interrupts, the required boundary is already decided:
  ISR sets a flag/queue only, the main loop polls it, and
  estimation/guidance/control code is never reachable from ISR context.
- **No dynamic allocation after init.** Every filter/controller/logger
  state is a fixed-size struct, stack- or static-allocated. One `.c`/`.h`
  pair per logical module.
- **Translation shims, never raw casts**, between any two same-concept-
  different-values enums/types (e.g. HAL status codes vs. sim fault
  codes).
- **Numeric regression gate.** The host-native test suite is run after
  every implementation step and the pass count is reported; a step isn't
  done until that gate is green.
- **V2 requires explicit confirmation.** Known debt (below) is tracked
  with IDs, not silently folded into V1 changes or started without the
  project owner's sign-off.

## `v_safe(range)`: shape and the glideslope analogy

```
v_safe(r) = clamp(k * sqrt(r), v_floor, v_cap)
```

Defaults for a ~1 m track: `k = 0.5`, `v_cap = 0.6 m/s` (caps the allowed
closing speed near the start of the approach, where `sqrt(r)` would
otherwise keep growing as `r` grows), `v_floor = 0.03 m/s` (floors the
target near contact so it approaches a small positive crawl rather than
exactly zero, avoiding a divide/derivative singularity right at dock).

This is the classic **glideslope** shape from real proximity-operations
guidance: allow a closing speed proportional to remaining range budget,
capped at a sane maximum, floored just above zero near contact. The real-
world analog is a cockpit glideslope indicator: it shows the pilot how far
the aircraft's current approach deviates from a target closing profile —
it does not itself fly the aircraft. This project's servo gauge has
exactly the same relationship to the cart: it visualizes deviation from
`v_safe(range)`, and has no authority over the cart's actual motion, just
as a glideslope needle has no authority over the aircraft's.

## V1 measurement results

All numbers below were captured from the current, passing test binaries
(`./build/tests/test_estimation_chain_metrics`,
`./build/tests/test_control_chain_metrics`), run directly against the
committed test scenarios — not reconstructed from memory or fabricated.

### Estimation chain (`docs/design.md` §8, `tests/test_estimation_chain_metrics.c`)

Scenario: real 1 m initial range / 1 m track, 100 Hz loop, 5 s runs
(500 ticks), gentle scripted accelerations chosen to stay inside the real
rig's range/`v_cap` envelope.

| Test | Result |
|---|---|
| Nominal-noise RMSE/bias/variance | RAW RMSE = **0.0816 m/s**, FUSED RMSE = **0.0078 m/s** (~10x better than RAW), FUSED bias = **-0.0035 m/s**, FUSED variance = **0.000048** |
| Dropout recovery (~300 ms ToF gap, tol. 0.1 m/s) | reconverges at **settle = 2.92 s** into the 5.0 s run |
| Step response (bounded pulse → 0.2 m/s plateau, tol. 0.05 m/s) | reconverges at **settle = 2.15 s**, i.e. **2.85 s** of real headroom before the 5.00 s run ends |
| IMU bias-drift boundedness (persistent 0.3 m/s² accel bias) | FUSED RMSE stays bounded at **0.1530 m/s** (does not diverge), because periodic ToF correction keeps resetting the drift |

The first row is the direct evidence for "ToF dominates long-term
accuracy": FUSED's RMSE against ground-truth velocity is roughly an order
of magnitude better than RAW's noisy differenced signal, because the
complementary filter is anchored back to ToF on every fresh sample.

### Control chain (`docs/design.md` §8, `tests/test_control_chain_metrics.c`)

Same 1 m/1 m rig envelope, same 100 Hz/5 s run shape; `speed_error`
computed against a *held/measured* range (last known `HAL_OK` ToF sample,
never ground truth — matching the real onboard-controller discipline of
never seeing anything a real sensor wouldn't have yet).

| Test | Result |
|---|---|
| RMS tracking error vs. `v_safe(range)` | tracking RMSE = **0.3377 m/s**, control-output variance = **0.0411**, saturation = **0.00%** of ticks (this gentle scenario never drives the PD output to the ±1 clamp) |
| D-term jitter, unfiltered vs. filtered (same run, raised `Kd`) | unfiltered jitter RMS = **0.2497**, filtered jitter RMS = **0.0409** — roughly a **6x reduction** from the EMA D-term filter alone |
| PD step response (step in `speed_error` at t = 1.0 s) | settles at **t = 1.13 s**, i.e. **0.13 s** after the step, final output = **0.45** |

The second row is the direct evidence for the D-term noise-amplification
point above, and for the fix: same run, same noise, only the filtering
differs.

## Build & test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Current state (verified by running the commands above): **19/19 ctest
suites passing, 100% tests passed** — 18 host-native C test binaries plus
`test_hal_boundary` (a shell-script structural check, pass/fail rather
than a ratio). Per-suite individual `TEST()` case counts:

| Suite | Cases |
|---|---|
| test_dt_validation | 4/4 |
| test_sim_noise | 3/3 |
| test_sim_cart | 4/4 |
| test_sim_range_sensor | 4/4 |
| test_sim_imu | 5/5 |
| test_hal_host | 4/4 |
| test_gravity_compensation | 2/2 |
| test_orientation_1d | 3/3 |
| test_raw_speed | 4/4 |
| test_metrics | 6/6 |
| test_complementary_filter | 5/5 |
| test_estimator | 5/5 |
| test_estimation_chain_metrics | 4/4 |
| test_v_safe | 4/4 |
| test_pd_controller | 5/5 |
| test_actuator_mapping | 4/4 |
| test_control_chain_metrics | 3/3 |
| test_csv_logger | 2/2 |
| test_hal_boundary | shell script, pass/fail |

**Total: 71/71 individual C test cases passing, plus the HAL-boundary
shell-script check — all green.**

To run the host-simulated pipeline itself and produce a CSV log
(`docs/design.md` §10 format), build and run `main_host_sim`:

```bash
./build/main_host_sim
```

## Known debt (V2 candidates)

V2 is **not started**. Per `CLAUDE.md` constraint 9 and
`docs/design.md` §12, none of the items below may be started without
explicit confirmation that V1 is complete, tested, and reviewed.

| ID | Item | Notes |
|---|---|---|
| DEBT-1 | ESP32 hardware backend (`hal_esp32.c`) + real servo output | The actual physical build. Requires the one-file hardware boundary (above) to already hold. |
| DEBT-2 | Kalman filter as an alternate estimator, compared side-by-side against the V1 complementary filter using the same §8 measurement contract | Explicitly not built in V1. |
| DEBT-3 | Richer timing realism: injected ToF latency, IMU timing jitter, out-of-order/stale measurement handling, latency metrics under these conditions | V1's dt validation handles pathological `dt` but does not inject or specifically characterize jitter/latency. |
| DEBT-4 | Actuator deadband + rate-limiting beyond basic saturation | V1 has saturation only. |

**V1 scope, for reference:** HAL interfaces + host backend, sim modules
(`sim_range_sensor`, `sim_imu`, `sim_cart`, `sim_noise`), the full
estimation chain (RAW + gravity compensation + 1D pitch + complementary
filter + dt validation), the full guidance/control chain (`v_safe` + PD
with filtered D-term + actuator mapping), CSV logging, and the host-native
CMake test suite covering the full measurement contract above plus
staleness/dt-validation edge cases.
