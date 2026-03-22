#include "board_config.h"
#include "sdio.h"

// Helper for GPIO setup
static void gpio_setup(uint pin, bool out) {
    gpio_init(pin);
    gpio_set_dir(pin, out);
}

void board_init(void) {
    // ---- System clock ----
    set_sys_clock_khz(PICO_SYSCLK, true);
    stdio_init_all();

    // ---- GPIO init ----
    gpio_setup(SD_CMD_PIN, false);
    gpio_pull_up(SD_CMD_PIN);

    gpio_setup(SD_CLK_PIN, false);

    // ---- SDIO PIO init ----
    sdio_init(pio0, SD_CMD_PIN, SD_CLK_PIN, SD_DA0_PIN);
}
