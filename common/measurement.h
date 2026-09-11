#ifndef COMMON_MEASUREMENT_H
#define COMMON_MEASUREMENT_H

#include "common/types.h"

typedef struct {
    double range_m;
    timestamp_t ts;
    hal_status_t status;
} range_sample_t;

typedef struct {
    double accel_mps2[3]; /* X=longitudinal, Y=lateral(unused), Z=vertical */
    double gyro_rps[3];   /* [1]=pitch rate, used; [0],[2] unused in V1 */
    timestamp_t ts;
    hal_status_t status;
} imu_sample_t;

#endif /* COMMON_MEASUREMENT_H */
