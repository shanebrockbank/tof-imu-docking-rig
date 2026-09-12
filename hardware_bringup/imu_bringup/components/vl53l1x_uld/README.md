# vl53l1x_uld

Vendored ST VL53L1X Ultra Lite Driver core (`VL53L1X_api.c/h`,
`VL53L1X_calibration.c/h`, `VL53L1X_error_codes.h`, `VL53L1X_types.h`),
copied unmodified from
[david-asher/VL53L1-ULD-ESP](https://github.com/david-asher/VL53L1-ULD-ESP)
(itself packaging ST's STSW-IMG009 Ultra Lite Driver). Two files from that
repo were deliberately **not** vendored because nothing in the core API
includes or calls them: `VL53L1X_def.h` (references a missing
`vl53l1_ll_def.h` from the full, non-lite driver) and
`VL53L1X_register_map.h` (a much larger reference register map;
`VL53L1X_api.h` already `#define`s the small subset of registers the core
API actually uses).

License: ST dual-licenses this code — "STMicroelectronics Proprietary
license" or "BSD 3-clause New/Revised License", at the licensee's option
(see the header comment in each vendored file). This project uses it under
the BSD 3-clause option.

`vl53l1_platform.h`/`.c` are **not** vendored — they're a from-scratch
implementation of the ULD core's ~9-function I2C seam
(`VL53L1_WrByte`/`RdByte`/`WriteMulti`/etc.), written against ESP-IDF's
`driver/i2c_master.h` so the ToF sensor can share the same
`i2c_master_bus_handle_t` the IMU already uses in `main.c`. The upstream
repo's own platform layer uses the legacy `driver/i2c.h` API, which
ESP-IDF does not allow to coexist with `i2c_master.h` on the same port —
sharing one bus/driver style across all sensors is also what
hardware_bringup Test 6 (combined I2C bus timing) needs to actually test.

Only single-device use is supported (call `vl53l1_platform_bind()` once);
the `dev` handle threaded through every `VL53L1X_*` call is accepted but
ignored.
