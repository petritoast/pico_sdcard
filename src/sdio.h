#ifndef SDIO_H
#define SDIO_H

#include <stdint.h>
#include "hardware/pio.h"

#define SD_DAT_MODE_4B_WR    0
#define SD_DAT_MODE_4B_RD    1

/** Initialise SDIO PIO state machines (CLK strobe + CMD capture) */
void sdio_set_dat_mode(uint8_t mode);
void sdio_init(PIO pio, uint cmd_pin, uint clk_pin, uint dat_pin_base);

/** Blocking read of one 16-bit autopush word from the CMD state machine */
int sdio_handle_cmd(uint32_t *arg_out, uint32_t *payload);

void sdio_cmd_respond(const uint8_t *bytes, size_t payload_len);

void sdio_send_block(uint32_t * buf);
int sdio_receive_block(uint32_t * buf);

#endif // SDIO_H
