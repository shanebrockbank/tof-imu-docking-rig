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
