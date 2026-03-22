#include <stdio.h>
#include <string.h>
#include "board_config.h"
#include "sdio.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "pico/time.h"

uint32_t buf_data[256] = {0};

const uint32_t mbr_bytes[16] = { // Copied from Pico uf2 MBR
    0xeb3c904d, 0x5357494e, 0x342e3100, 0x02080100, // 16
    0x02000200, 0x00f88100, 0x01000100, 0x01000000, // 32
    0xffff0300, 0x0000292c, 0xb0253252, 0x50323335, // 48
    0x30202020, 0x20204641, 0x54313620, 0x2020EBFE  // 64
};

void block_handler(uint32_t *buf, uint32_t lba) {
    memset(buf, 0, 512);

    if (lba == 0) {
        memcpy( buf, mbr_bytes, sizeof(mbr_bytes) );
        buf[127] = 0x000055AA;  // MBR signature
    } else if (lba == 1) {
        buf[0] = 0xF8FFFFFF;
        buf[1] = 0xFFFFFFFF;
    }
}

/* Core 1: Time-Critical SDIO Logic
 * This core handles the SDIO state machine and PIO transfers */
void core1_main() {
    irq_set_mask_enabled(0xFFFFFFFF, false);

    uint32_t *sd_payload = &buf_data[0];
    while (true) {
        uint32_t cmd_arg;
        int cmd_num = sdio_handle_cmd(&cmd_arg, sd_payload);
        
        if (cmd_num < 0) continue;

        // Report CMD: Type 0x0 [Tag:4] [Cmd:12] [ArgLower:16]
        multicore_fifo_push_blocking((0x0u << 28) | ((uint32_t)cmd_num << 16) | (cmd_arg & 0xFFFF));

        // Read Handling (CMD17 & CMD18)
        if (cmd_num == 17 || cmd_num == 18) {
            uint32_t current_lba = cmd_arg;
            while(true) {
                block_handler(sd_payload, current_lba);
                sdio_send_block(sd_payload);

                // Report READ: Type 0x2 [Tag:4] [LBA:12] [Data:16]
                multicore_fifo_push_blocking((0x2u << 28) | ((current_lba & 0xFFF) << 16) | (sd_payload[0] >> 16));

                if (cmd_num == 17) break; 

                uint32_t next_arg;
                int next_cmd = sdio_handle_cmd(&next_arg, sd_payload);
                if (next_cmd != -1) {
                    multicore_fifo_push_blocking((0x0u << 28) | ((uint32_t)next_cmd << 16) | (next_arg & 0xFFFF));
                    break;
                }
                current_lba++;
            }
        }

        // Write Handling (CMD24 & CMD25)
        if (cmd_num == 24 || cmd_num == 25) {
            uint32_t current_lba = cmd_arg;
            sdio_set_dat_mode(SD_DAT_MODE_4B_RD); 

            while(true) {
                block_handler(sd_payload, 256); // 0s
                int status = sdio_receive_block(sd_payload);
                if (status != 0) {
                    multicore_fifo_push_blocking((0x0u << 28) | ((uint32_t)status << 16) | 0xFFFF);
                    break;
                };

                // Report WRITE: Type 0x1 [Tag:4] [LBA:12] [Data:16]
                multicore_fifo_push_blocking((0x1u << 28) | ((current_lba & 0xFFF) << 16) | (sd_payload[0] >> 16));

                if (cmd_num == 24) break; 
                current_lba++;
            }
            sdio_set_dat_mode(SD_DAT_MODE_4B_WR); 
        }

    }
}


/* Core 0: Manager & Logging
 * Handles board init, USB stack, and prints logs from Core 1 */
int main(void) {
    board_init();

    // Launch the state machine on Core 1
    multicore_launch_core1(core1_main);

    while (true) {
        // Poll for log events from the SDIO core
        while (multicore_fifo_rvalid()) {
            uint32_t val = multicore_fifo_pop_blocking();
            uint8_t type = val >> 28;
            uint16_t head = (val >> 16) & 0xFFF;
            uint16_t tail = val & 0xFFFF;
            
            if (val == 0) printf("reset ");
            else if (type == 0x0) printf("C%ux%X ", head, tail);
            //else if (type == 0x1) printf("W%u:%04X ", head, tail);
            //else if (type == 0x2) printf("R%u:%04X ", head, tail);
            //else printf("?%08X ", (unsigned int)val);
        }
        
        fflush(stdout);
        sleep_us(1);
    }
}
