idf_component_register(
    SRCS "main_esp32.c" "../../hal/hal_esp32.c"
    INCLUDE_DIRS "../.."
    PRIV_REQUIRES esp_timer
)
