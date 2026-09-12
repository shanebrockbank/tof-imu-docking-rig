#include "hal/hal_esp32.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "logging/csv_logger.h"
#include "common/dt_validation.h"

void app_main(void) {
    hal_t h = hal_esp32_create();

    estimator_t estimator;
    estimator_init(&estimator, 0.90);

    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    csv_logger_t log;
    csv_logger_open_stream(&log, stdout);

    timestamp_t prev_now = h.clock_now(h.ctx);
    double last_measured_range_m = 0.0;
    bool have_measured_range = false;

    for (;;) {
        h.pace_tick(h.ctx);
        timestamp_t now = h.clock_now(h.ctx);
        double ctrl_dt = dt_between(prev_now, now);

        range_sample_t range_sample;
        imu_sample_t imu_sample;
        hal_status_t range_status = h.range_read(h.ctx, &range_sample);
        hal_status_t imu_status = h.imu_read(h.ctx, &imu_sample);
        range_sample.status = range_status;
        imu_sample.status = imu_status;

        if (range_status == HAL_OK) {
            last_measured_range_m = range_sample.range_m;
            have_measured_range = true;
        }

        estimator_output_t est = estimator_tick(&estimator, &range_sample, &imu_sample);

        /* Unlike main_host_sim.c (which seeds an initial known range from the
           simulator and never gates on this), real hardware has no ground-truth
           initial range — so control/actuation/logging are deliberately gated
           on having seen at least one valid ToF sample. This is an intentional,
           acknowledged departure from an otherwise-identical loop shape between
           the two mains. */
        if (have_measured_range) {
            double target = v_safe(last_measured_range_m);
            double speed_error = est.fused_speed_mps - target;
            pd_output_t ctrl = pd_controller_update(&pd, speed_error, ctrl_dt);
            double servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered);
            h.actuator_set_angle_deg(h.ctx, servo_deg);

            csv_logger_write_row_hw(&log,
                now.t_s,
                last_measured_range_m,
                est.raw_speed.value_mps,
                est.raw_speed.is_stale,
                est.fused_speed_mps,
                target,
                speed_error,
                ctrl.control_output_unfiltered,
                ctrl.control_output_filtered,
                servo_deg);
        }

        prev_now = now;
    }
}
