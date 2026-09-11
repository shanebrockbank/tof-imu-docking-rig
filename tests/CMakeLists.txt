add_library(common_lib STATIC
    ${CMAKE_SOURCE_DIR}/common/dt_validation.c
)

add_executable(test_dt_validation test_dt_validation.c)
target_link_libraries(test_dt_validation common_lib)
add_test(NAME test_dt_validation COMMAND test_dt_validation)

add_library(sim_noise_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_noise.c)
target_link_libraries(sim_noise_lib m)

add_executable(test_sim_noise test_sim_noise.c)
target_link_libraries(test_sim_noise sim_noise_lib m)
add_test(NAME test_sim_noise COMMAND test_sim_noise)

add_library(sim_cart_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_cart.c)

add_executable(test_sim_cart test_sim_cart.c)
target_link_libraries(test_sim_cart sim_cart_lib)
add_test(NAME test_sim_cart COMMAND test_sim_cart)

add_library(sim_range_sensor_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_range_sensor.c)
target_link_libraries(sim_range_sensor_lib sim_noise_lib m)

add_executable(test_sim_range_sensor test_sim_range_sensor.c)
target_link_libraries(test_sim_range_sensor sim_range_sensor_lib m)
add_test(NAME test_sim_range_sensor COMMAND test_sim_range_sensor)

add_library(sim_imu_lib STATIC ${CMAKE_SOURCE_DIR}/sim/sim_imu.c)
target_link_libraries(sim_imu_lib sim_noise_lib m)

add_executable(test_sim_imu test_sim_imu.c)
target_link_libraries(test_sim_imu sim_imu_lib m)
add_test(NAME test_sim_imu COMMAND test_sim_imu)

add_library(hal_host_lib STATIC ${CMAKE_SOURCE_DIR}/hal/hal_host.c)
target_link_libraries(hal_host_lib sim_cart_lib sim_range_sensor_lib sim_imu_lib m)

add_executable(test_hal_host test_hal_host.c)
target_link_libraries(test_hal_host hal_host_lib m)
add_test(NAME test_hal_host COMMAND test_hal_host)

add_library(gravity_compensation_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/gravity_compensation.c)
target_link_libraries(gravity_compensation_lib m)

add_executable(test_gravity_compensation test_gravity_compensation.c)
target_link_libraries(test_gravity_compensation gravity_compensation_lib sim_imu_lib m)
add_test(NAME test_gravity_compensation COMMAND test_gravity_compensation)

add_library(orientation_1d_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/orientation_1d.c)
target_link_libraries(orientation_1d_lib common_lib)

add_executable(test_orientation_1d test_orientation_1d.c)
target_link_libraries(test_orientation_1d orientation_1d_lib)
add_test(NAME test_orientation_1d COMMAND test_orientation_1d)

add_library(raw_speed_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/raw_speed.c)
target_link_libraries(raw_speed_lib common_lib)

add_executable(test_raw_speed test_raw_speed.c)
target_link_libraries(test_raw_speed raw_speed_lib)
add_test(NAME test_raw_speed COMMAND test_raw_speed)

add_library(metrics_lib STATIC ${CMAKE_SOURCE_DIR}/common/metrics.c)
target_link_libraries(metrics_lib m)

add_executable(test_metrics test_metrics.c)
target_link_libraries(test_metrics metrics_lib m)
add_test(NAME test_metrics COMMAND test_metrics)

add_library(complementary_filter_lib STATIC ${CMAKE_SOURCE_DIR}/estimation/complementary_filter.c)
target_link_libraries(complementary_filter_lib common_lib)

add_executable(test_complementary_filter test_complementary_filter.c)
target_link_libraries(test_complementary_filter complementary_filter_lib)
add_test(NAME test_complementary_filter COMMAND test_complementary_filter)
