cmake_minimum_required(VERSION 3.16)
project(tof_imu_docking_rig C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})

enable_testing()
add_subdirectory(tests)
