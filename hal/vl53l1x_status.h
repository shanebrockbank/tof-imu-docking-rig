#ifndef HAL_VL53L1X_STATUS_H
#define HAL_VL53L1X_STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include "common/types.h"

/* Translates the VL53L1X ULD's own (data-ready flag, range_status byte)
   pair into this project's hal_status_t — an explicit translator per
   CLAUDE.md constraint 7 (never a raw cast between same-concept,
   different-value status codes). range_status == 0 is the ULD's own
   VL53L1_RANGESTATUS_RANGE_VALID; any other value is a fault (out of
   range, signal failure, etc. — see ST's VL53L1X_error_codes.h). */
hal_status_t vl53l1x_translate_status(bool data_ready, uint8_t range_status);

#endif /* HAL_VL53L1X_STATUS_H */
