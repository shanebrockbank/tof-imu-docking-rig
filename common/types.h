#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

typedef struct {
    double t_s; /* monotonic seconds, arbitrary epoch */
} timestamp_t;

typedef enum {
    HAL_OK = 0,
    HAL_NO_NEW_DATA,
    HAL_FAULT,
} hal_status_t;

#endif /* COMMON_TYPES_H */
