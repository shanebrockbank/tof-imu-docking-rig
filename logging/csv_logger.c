#include "logging/csv_logger.h"

bool csv_logger_open(csv_logger_t *log, const char *path) {
    log->fp = fopen(path, "w");
    if (!log->fp) return false;
    fprintf(log->fp,
        "t_s,true_range_m,true_velocity_mps,raw_speed_mps,raw_is_stale,"
        "fused_speed_mps,v_safe_mps,speed_error_mps,"
        "control_output_unfiltered,control_output_filtered,servo_angle_deg\n");
    return true;
}

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
                           double servo_angle_deg) {
    if (!log->fp) return;
    fprintf(log->fp, "%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f\n",
        t_s, true_range_m, true_velocity_mps, raw_speed_mps, raw_is_stale ? 1 : 0,
        fused_speed_mps, v_safe_mps, speed_error_mps,
        control_output_unfiltered, control_output_filtered, servo_angle_deg);
}

void csv_logger_close(csv_logger_t *log) {
    if (log->fp) {
        fclose(log->fp);
        log->fp = NULL;
    }
}
