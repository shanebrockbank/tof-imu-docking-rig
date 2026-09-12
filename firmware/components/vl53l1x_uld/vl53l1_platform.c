/*
 * vl53l1_platform.c — see vl53l1_platform.h for context. From-scratch
 * implementation against ESP-IDF's i2c_master.h driver, not vendored.
 *
 * VL53L1X register access convention: a 16-bit register index, sent
 * big-endian, immediately followed (write) or by a repeated-start read
 * (read) with the data bytes.
 */

#include "vl53l1_platform.h"

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS   100
#define MAX_WRITE_PAYLOAD 32 /* largest real call in VL53L1X_api.c is a single byte/word/dword */

static i2c_master_dev_handle_t s_dev;
static uint32_t s_error_count;

void vl53l1_platform_bind(i2c_master_bus_handle_t bus, uint8_t i2c_7bit_addr) {
  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = i2c_7bit_addr,
    .scl_speed_hz = 400000,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev));
  s_error_count = 0;
}

uint32_t vl53l1_platform_get_error_count(void) { return s_error_count; }
void vl53l1_platform_reset_error_count(void) { s_error_count = 0; }

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  (void)dev;
  if (count > MAX_WRITE_PAYLOAD) return -1;
  uint8_t buf[2 + MAX_WRITE_PAYLOAD];
  buf[0] = (uint8_t)(index >> 8);
  buf[1] = (uint8_t)(index & 0xFF);
  memcpy(&buf[2], pdata, count);
  if (i2c_master_transmit(s_dev, buf, count + 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK) return 0;
  s_error_count++;
  return -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  (void)dev;
  // Zero the destination before the transfer so a failed/partial transaction
  // deterministically yields zero bytes rather than the caller's stale stack
  // memory -- the ULD core doesn't check every read's return status (e.g.
  // VL53L1X_CheckForDataReady ignores VL53L1_RdByte's result), so garbage
  // here could otherwise silently misdrive its ready-flag logic.
  memset(pdata, 0, count);
  uint8_t reg[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
  if (i2c_master_transmit_receive(s_dev, reg, 2, pdata, count, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK) return 0;
  s_error_count++;
  return -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
  return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
  uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
  return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
  uint8_t buf[4] = {
    (uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data
  };
  return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) {
  return VL53L1_ReadMulti(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata) {
  uint8_t buf[2];
  int8_t r = VL53L1_ReadMulti(dev, index, buf, 2);
  *pdata = ((uint16_t)buf[0] << 8) | buf[1];
  return r;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata) {
  uint8_t buf[4];
  int8_t r = VL53L1_ReadMulti(dev, index, buf, 4);
  *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
  return r;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
  (void)dev;
  vTaskDelay(pdMS_TO_TICKS(wait_ms));
  return 0;
}
