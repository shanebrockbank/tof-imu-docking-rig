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
