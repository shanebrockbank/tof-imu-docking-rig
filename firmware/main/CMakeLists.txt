idf_component_register(
    SRCS "main_esp32.c"
         "../../hal/hal_esp32.c"
         "../../hal/vl53l1x_status.c"
         "../../common/dt_validation.c"
         "../../common/metrics.c"
         "../../estimation/gravity_compensation.c"
         "../../estimation/orientation_1d.c"
         "../../estimation/raw_speed.c"
         "../../estimation/complementary_filter.c"
         "../../estimation/estimator.c"
         "../../guidance/v_safe.c"
         "../../guidance/pd_controller.c"
         "../../guidance/actuator_mapping.c"
         "../../logging/csv_logger.c"
    INCLUDE_DIRS "../.."
    REQUIRES vl53l1x_uld
    PRIV_REQUIRES esp_timer esp_rom driver freertos
)
