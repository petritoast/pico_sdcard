#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/clocks.h"

// ---- System Clock ----
#define PICO_SYSCLK    250000

// ---- SDIO Pin Assignments ----
#define SD_CMD_PIN        0
#define SD_CLK_PIN        1
#define SD_DA0_PIN        2
#define SD_DA1_PIN        3
#define SD_DA2_PIN        4
#define SD_DA3_PIN        5

/**
 * Initialise system clock, GPIOs, and subsystems.
 * Call once at startup before entering the main loop.
 */
void board_init(void);

#endif // BOARD_CONFIG_H
