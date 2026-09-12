idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c" "../../hal/vl53l1x_status.c"
    INCLUDE_DIRS "../.."
    REQUIRES vl53l1x_uld
    PRIV_REQUIRES esp_timer esp_rom driver freertos
)
