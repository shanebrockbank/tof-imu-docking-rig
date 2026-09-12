#include "hal/hal_esp32.h"

void app_main(void) {
    hal_t h = hal_esp32_create();
    for (;;) {
        h.pace_tick(h.ctx);
        (void)h.clock_now(h.ctx);
    }
}
