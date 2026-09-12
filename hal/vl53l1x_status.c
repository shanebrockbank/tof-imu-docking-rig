#include "hal/vl53l1x_status.h"

hal_status_t vl53l1x_translate_status(bool data_ready, uint8_t range_status) {
    if (!data_ready) return HAL_NO_NEW_DATA;
    return (range_status == 0) ? HAL_OK : HAL_FAULT;
}
