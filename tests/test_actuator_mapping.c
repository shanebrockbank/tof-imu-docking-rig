#include "tests/test_framework.h"
#include "guidance/actuator_mapping.h"

TEST(test_zero_control_output_maps_to_center) {
    CHECK_NEAR(actuator_map_to_servo_deg(0.0), 90.0, 1e-9);
}

TEST(test_plus_one_maps_to_upper_bound) {
    CHECK_NEAR(actuator_map_to_servo_deg(1.0), 135.0, 1e-9);
}

TEST(test_minus_one_maps_to_lower_bound) {
    CHECK_NEAR(actuator_map_to_servo_deg(-1.0), 45.0, 1e-9);
}

TEST(test_out_of_range_input_is_clamped_before_mapping) {
    CHECK_NEAR(actuator_map_to_servo_deg(2.0), 135.0, 1e-9);
    CHECK_NEAR(actuator_map_to_servo_deg(-2.0), 45.0, 1e-9);
}

int main(void) {
    RUN_TEST(test_zero_control_output_maps_to_center);
    RUN_TEST(test_plus_one_maps_to_upper_bound);
    RUN_TEST(test_minus_one_maps_to_lower_bound);
    RUN_TEST(test_out_of_range_input_is_clamped_before_mapping);
    TEST_SUMMARY();
    return 0;
}
