/*
 * vl53l1_platform.h
 *
 * Platform seam for ST's vendored VL53L1X Ultra Lite Driver (VL53L1X_api.c/h,
 * VL53L1X_calibration.c/h, VL53L1X_error_codes.h, VL53L1X_types.h —
 * unmodified ST source, copied from
 * https://github.com/david-asher/VL53L1-ULD-ESP, itself packaging ST's
 * STSW-IMG009 Ultra Lite Driver).
 *
 * This file (and vl53l1_platform.c) is NOT vendored — it's a from-scratch
 * implementation of the ~9-function I2C seam the ULD core expects, written
 * against ESP-IDF's new i2c_master.h driver so the ToF sensor can share the
 * same i2c_master_bus_handle_t the IMU already uses in main.c (required for
 * hardware_bringup Test 6, combined-bus timing, to mean anything). The
 * upstream repo's own platform layer uses the legacy driver/i2c.h API, which
 * ESP-IDF does not allow to coexist with i2c_master.h on the same port.
 *
 * Call vl53l1_platform_bind() once, after the shared bus is created and
 * before any VL53L1X_* API call. Only one device is supported (this
 * bring-up firmware has exactly one ToF sensor) — the `dev` handle threaded
 * through every VL53L1X_* call is accepted but ignored.
 */

#ifndef VL53L1_PLATFORM_H_
#define VL53L1_PLATFORM_H_

#include <stdint.h>

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Registers the ToF sensor as a device on an already-initialized shared
 * bus. i2c_7bit_addr is the sensor's plain 7-bit I2C address (0x29 default),
 * not pre-shifted. */
void vl53l1_platform_bind(i2c_master_bus_handle_t bus, uint8_t i2c_7bit_addr);

/* Count of failed I2C transactions (timeouts, NACKs) since bind or the last
 * reset. The ULD core itself does not check every call's status (e.g.
 * VL53L1X_CheckForDataReady ignores VL53L1_RdByte's return value), so a
 * transient bus error can otherwise pass silently — callers that want to
 * report reliability (as Tests 4/5/6 do for the IMU) should check this. */
uint32_t vl53l1_platform_get_error_count(void);
void vl53l1_platform_reset_error_count(void);

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1_PLATFORM_H_ */
