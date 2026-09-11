cmake_minimum_required(VERSION 3.16)
project(tof_imu_docking_rig C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})

enable_testing()
add_subdirectory(tests)

add_executable(main_host_sim
    main_host_sim.c
    common/dt_validation.c
    common/metrics.c
    sim/sim_noise.c
    sim/sim_cart.c
    sim/sim_range_sensor.c
    sim/sim_imu.c
    hal/hal_host.c
    estimation/gravity_compensation.c
    estimation/orientation_1d.c
    estimation/raw_speed.c
    estimation/complementary_filter.c
    estimation/estimator.c
    guidance/v_safe.c
    guidance/pd_controller.c
    guidance/actuator_mapping.c
    logging/csv_logger.c
)
target_link_libraries(main_host_sim m)
