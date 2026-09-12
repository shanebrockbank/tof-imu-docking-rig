#ifndef HAL_HAL_ESP32_H
#define HAL_HAL_ESP32_H

#include "hal/hal.h"

/* Populates and returns a hal_t bound to the real ESP32 peripherals (I2C
   bus, ICM20948, VL53L1X, servo PWM, esp_timer clock). Zero ESP-IDF types
   appear in this header — see CLAUDE.md constraint 1. Call once at
   startup, from app_main() in main_esp32.c. Backend state is file-static
   inside hal_esp32.c (there is exactly one board), so this takes no
   arguments, unlike hal_host_create(). */
hal_t hal_esp32_create(void);

#endif /* HAL_HAL_ESP32_H */
