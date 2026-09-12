#include "tests/test_framework.h"
#include "hal/vl53l1x_status.h"

TEST(test_not_ready_is_no_new_data) {
    CHECK(vl53l1x_translate_status(false, 0) == HAL_NO_NEW_DATA);
    CHECK(vl53l1x_translate_status(false, 7) == HAL_NO_NEW_DATA);
}

TEST(test_ready_and_valid_is_ok) {
    CHECK(vl53l1x_translate_status(true, 0) == HAL_OK);
}

TEST(test_ready_and_nonzero_status_is_fault) {
    CHECK(vl53l1x_translate_status(true, 1) == HAL_FAULT);
    CHECK(vl53l1x_translate_status(true, 255) == HAL_FAULT);
}

int main(void) {
    RUN_TEST(test_not_ready_is_no_new_data);
    RUN_TEST(test_ready_and_valid_is_ok);
    RUN_TEST(test_ready_and_nonzero_status_is_fault);
    TEST_SUMMARY();
    return 0;
}
