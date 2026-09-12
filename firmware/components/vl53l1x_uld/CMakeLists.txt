# VL53L1X Ultra Lite Driver (ST source, vendored) + a from-scratch
# i2c_master.h platform shim (not vendored). See include/vl53l1_platform.h.
idf_component_register(
    SRCS "VL53L1X_api.c" "VL53L1X_calibration.c" "vl53l1_platform.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
