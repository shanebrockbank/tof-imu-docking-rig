#ifndef LOGGING_CSV_LOGGER_H
#define LOGGING_CSV_LOGGER_H

#include <stdio.h>
#include <stdbool.h>

typedef struct {
    FILE *fp;
} csv_logger_t;

/* Opens path for writing and writes the header row immediately. Returns
   false on failure to open (fp left NULL). */
bool csv_logger_open(csv_logger_t *log, const char *path);

/* Column order fixed by docs/design.md §10 — do not reorder without
   updating the design doc and README together. */
void csv_logger_write_row(csv_logger_t *log,
                           double t_s,
                           double true_range_m,
                           double true_velocity_mps,
                           double raw_speed_mps,
                           bool raw_is_stale,
                           double fused_speed_mps,
                           double v_safe_mps,
                           double speed_error_mps,
                           double control_output_unfiltered,
                           double control_output_filtered,
                           double servo_angle_deg);

void csv_logger_close(csv_logger_t *log);

#endif /* LOGGING_CSV_LOGGER_H */
