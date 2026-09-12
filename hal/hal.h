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

    /* Blocks until the next main-loop tick boundary (real backends); a
       no-op on backends with no wall clock to wait on (e.g. host/sim).
       Called once per loop iteration by both main_host_sim.c and
       main_esp32.c so their loop bodies are identical in shape — see
       docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md §4. */
    void (*pace_tick)(void *ctx);
} hal_t;

#endif /* HAL_HAL_H */
