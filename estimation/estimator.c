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
