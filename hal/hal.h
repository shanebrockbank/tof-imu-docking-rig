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
