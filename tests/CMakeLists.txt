add_library(common_lib STATIC
    ${CMAKE_SOURCE_DIR}/common/dt_validation.c
)

add_executable(test_dt_validation test_dt_validation.c)
target_link_libraries(test_dt_validation common_lib)
add_test(NAME test_dt_validation COMMAND test_dt_validation)
