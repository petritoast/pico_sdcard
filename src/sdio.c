#include <stdio.h>
#include "pico/stdlib.h"
#include "sdio.h"
#include "sd_clk_strobe.pio.h"
#include "sd_cmd.pio.h"
#include "sd_dat_tx.pio.h"
#include "sd_dat_rx.pio.h"

#define SM_CLK_STROBE  0
#define SM_CMD         1
#define SM_DAT         2

#define CMD_BIT_COUNT  55 // CMD frame bits (48 bits) + resp wait time (1-8 bits)

static PIO sdio_pio;
static const pio_program_t * loaded_dat_program = NULL;
static uint loaded_dat_offset = 0;
static uint _dat_pin_base;
static bool is_app_cmd = false;

// Pre-formatted responses. Format: [Index/Header] [Data 0..N] [CRC7 + Stop bit]
// 48-bit responses (6 bytes total)
uint8_t sd_resp_r1[6] = {0x06, 0x00, 0x00, 0x09, 0x20, 0x01}; // Template for R1
uint8_t sd_resp_r3[6] = {0x3F, 0xC0, 0xFF, 0x00, 0x00, 0xFF}; // OCR (CMD41)
uint8_t sd_resp_r6[6] = {0x03, 0xDE, 0xAD, 0x09, 0x20, 0x01}; // RCA + Status (CMD3)
uint8_t sd_resp_r7[6] = {0x08, 0x00, 0x00, 0x01, 0xAA, 0x01}; // CMD8 Echo

// Static CID/CSD data (17+1 bytes)
uint8_t sd_resp_cid[18] = {
    0x3F, // RSRVD
    0x03, 0x53, 0x44, // MID + OID
    'E', 'M', 'U', 'L', 'A', // PNM
    0x10, 0xDE, 0xAD, 0xBE, 0xEF, // PRV + PSN
    0x01, 0x63,
    0x01, 0xFF // END + padding
};
uint8_t sd_resp_csd[18] = {
    0x3F, // RSRVD
    0x40, 0x0E, 0x00,
    0x5A, // 0x32 (25MHz) or 0x5A (50MHz)
    0x7B, 0x59, // CCC + BL_LEN
    0x00, 0x00, 0x1F, 0xFF, // C_SIZE = 0x1FFF (8191+1 blocks of 512 bytes = ~8GB)
    0x7F, 0x80,
    0x0A, 0x40, 0x00,
    0x01, 0xFF // END + padding
};


// Table lookup for calculating CRC-7 checksum that is used in SDIO command packets.
static const uint8_t crc7_table[256] = {
	0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e,	0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
	0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c,	0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
	0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a,	0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
	0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28,	0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
	0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6,	0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
	0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84,	0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
	0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2,	0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
	0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0,	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
	0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc,	0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
	0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce,	0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
	0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98,	0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
	0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa,	0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
	0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34,	0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
	0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06,	0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
	0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50,	0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
	0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62,	0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2
};

static uint8_t sdio_crc7(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc7_table[crc ^ data[i]];
    }
    return (crc & 0xFE) | 0x01;
}


/**
 * Reconfigures the SM_DAT state machine for a specific operation mode.
 * @param mode SD_DAT_MODE_4B_WR (Card RX) or SD_DAT_MODE_4B_RD (Card TX)
 */
void sdio_set_dat_mode(uint8_t mode) {
    pio_sm_set_enabled(sdio_pio, SM_DAT, false);

    // Remove the current DAT program if one is loaded
    if (loaded_dat_program != NULL) {
        pio_remove_program(sdio_pio, loaded_dat_program, loaded_dat_offset);
        loaded_dat_program = NULL;
    }

    const pio_program_t * program = (mode == SD_DAT_MODE_4B_WR) ? &sd_dat_tx_program : &sd_dat_rx_program;
    loaded_dat_offset = pio_add_program(sdio_pio, program);
    loaded_dat_program = program;

    pio_sm_config c;
    if (mode == SD_DAT_MODE_4B_WR) c = sd_dat_tx_program_get_default_config(loaded_dat_offset);
    else c = sd_dat_rx_program_get_default_config(loaded_dat_offset);

    // Common configurations for both directions
    sm_config_set_jmp_pin(&c, _dat_pin_base);
    sm_config_set_set_pins(&c, _dat_pin_base, 4);
    sm_config_set_out_pins(&c, _dat_pin_base, 4);
    sm_config_set_in_pins(&c, _dat_pin_base);

    pio_sm_init(sdio_pio, SM_DAT, loaded_dat_offset, &c);

    if (mode == SD_DAT_MODE_4B_RD) { // Card Receives
        uint16_t nibble_count = 1024 + 16 - 1;
        uint8_t token_count = 8 - 1;
        uint8_t token_pattern = 0b11100101;
        uint32_t orch = (nibble_count << 16) | (token_count << 8) | token_pattern;

        pio_sm_put_blocking(sdio_pio, SM_DAT, orch);
        pio_sm_exec(sdio_pio, SM_DAT, pio_encode_pull(false, true));
        pio_sm_exec(sdio_pio, SM_DAT, pio_encode_mov(pio_y, pio_osr));
    }

    pio_sm_set_enabled(sdio_pio, SM_DAT, true);
}


void sdio_init(PIO pio, uint cmd_pin, uint clk_pin, uint dat_pin_base) {
    sdio_pio = pio;
    _dat_pin_base = dat_pin_base;

    // PIO GPIO Initialization
    pio_gpio_init(sdio_pio, clk_pin);
    pio_gpio_init(sdio_pio, cmd_pin);
    for(int i=0; i<4; i++) pio_gpio_init(sdio_pio, dat_pin_base + i);
    pio_sm_set_consecutive_pindirs(sdio_pio, SM_DAT, dat_pin_base, 4, false); // start as input
    hw_set_bits(&sdio_pio->input_sync_bypass, 1u << clk_pin); // See CLK before CMD/DAT lines

    // ---- CLK strobe (rising-edge detector) ----
    uint off_strobe = pio_add_program(sdio_pio, &sd_clk_strobe_program);
    pio_sm_config c_strb = sd_clk_strobe_program_get_default_config(off_strobe);
    sm_config_set_in_pins(&c_strb, clk_pin);
    pio_sm_init(sdio_pio, SM_CLK_STROBE, off_strobe, &c_strb);
    pio_sm_set_enabled(sdio_pio, SM_CLK_STROBE, true);

    // ---- CMD capture ----
    uint off_cmd = pio_add_program(sdio_pio, &sd_cmd_program);
    pio_sm_config c_cmd = sd_cmd_program_get_default_config(off_cmd);
    sm_config_set_in_pins(&c_cmd, cmd_pin);
    sm_config_set_jmp_pin(&c_cmd, cmd_pin);
    sm_config_set_out_pins(&c_cmd, cmd_pin, 1);
    sm_config_set_set_pins(&c_cmd, cmd_pin, 1);
    pio_sm_init(sdio_pio, SM_CMD, off_cmd, &c_cmd);
    
    // Preload bit counter into Y register
    pio_sm_put_blocking(sdio_pio, SM_CMD, CMD_BIT_COUNT);
    pio_sm_exec(sdio_pio, SM_CMD, pio_encode_pull(false, true));
    pio_sm_exec(sdio_pio, SM_CMD, pio_encode_mov(pio_y, pio_osr));
    pio_sm_exec(sdio_pio, SM_CMD, pio_encode_out(pio_y, 32));
    pio_sm_set_enabled(sdio_pio, SM_CMD, true);

    // ---- DAT read/write ----
    sdio_set_dat_mode(SD_DAT_MODE_4B_WR);

    // Initialize pre-formatted responses
    sd_resp_cid[16] = sdio_crc7(&sd_resp_cid[1], 15);
    sd_resp_csd[16] = sdio_crc7(&sd_resp_csd[1], 15);
    sd_resp_r1[5]   = sdio_crc7(sd_resp_r1, 5);
    sd_resp_r6[5]   = sdio_crc7(sd_resp_r6, 5);
    sd_resp_r7[5]   = sdio_crc7(sd_resp_r7, 5);
}


void sdio_cmd_respond(const uint8_t *bytes, size_t payload_len) {
    // First 16-bit word: Cycle count (Total bits - 1)
    uint16_t cycles = (payload_len * 8) - 1;
    pio_sm_put_blocking(sdio_pio, SM_CMD, (uint32_t)cycles << 16);

    // Send payload in 16-bit words
    for (size_t i = 0; i < payload_len; i=i+2) {
        uint32_t word = (uint32_t)bytes[i] << 24 | (uint32_t)bytes[i+1] << 16;
        pio_sm_put_blocking(sdio_pio, SM_CMD, word);
    }
}


// Calculate the CRC16 checksum for parallel 4 bit lines separately.
// When the SDIO bus operates in 4-bit mode, the CRC16 algorithm
// is applied to each line separately and generates total of
// 4 x 16 = 64 bits of checksum.
__attribute__((optimize("O3")))
static uint64_t sdio_crc16_4bit(uint32_t * data, uint16_t size) {
    uint64_t crc = 0;

    for(int i = 0; i < size; i++) {
        uint32_t data_in = data[i];

        uint32_t data_out = crc >> 32;
        crc = crc << 32;
        
        uint64_t xor32 = data_out ^ data_in;
        xor32 ^= (xor32 >> 16);
        uint64_t xor64 = xor32;
        crc ^= xor64;
        crc ^= xor64 << (5 * 4);
        crc ^= xor64 << (12 * 4);
    }
    
    return crc;
}


/**
 * Data block sender for SD_DAT lines (4-bit mode)
 */
static void sdio_dat_send(uint32_t *data_ptr, uint32_t n) {
    // 1. Calculate CRC16 for the 4-bit bus
    uint64_t crc = sdio_crc16_4bit(data_ptr, n);

    // Total words = 1 Start + N Data + 2 CRC + 1 Stop = N + 4 words.
    pio_sm_put_blocking(sdio_pio, SM_DAT, ((n + 4) * 8) - 1);
    pio_sm_put_blocking(sdio_pio, SM_DAT, 0xFFFFFFF0); // Start Nibble

    for (uint32_t i = 0; i < n; i++) {
        pio_sm_put_blocking(sdio_pio, SM_DAT, data_ptr[i]);
    }

    pio_sm_put_blocking(sdio_pio, SM_DAT, (uint32_t)(crc >> 32));
    pio_sm_put_blocking(sdio_pio, SM_DAT, (uint32_t)(crc & 0xFFFFFFFF));
    pio_sm_put_blocking(sdio_pio, SM_DAT, 0xFFFFFFFF); // Stop Nibble
}


static void sdio_scr_response(uint32_t * payload) {
    // 64 bits of SCR data
    payload[0] = 0x02158000; // SD Spec Version 3.0, 4-bit bus
    payload[1] = 0x00000000; // No special features

    sdio_dat_send(payload, 2);
}


static void sdio_swfn_status(uint32_t * payload, uint8_t set_mode) {
    uint16_t curr = 70; // in mA
    payload[0] = curr << 16 | 0x8001;
    payload[1] = 0x8001 << 16 | 0x8001;
    payload[2] = 0x8001 << 16 | 0x8001;
    payload[3] = 0x8003 << 16 | 0x0 << 12 | 0x0 << 8 | 0x0 << 4 | 0x0;
    payload[4] = 0x0 << 28 | 0 << 24 | 0x0 << 16 | 0x0000;
    for(int i = 5; i < 16; i++) payload[i] = 0; // Padding to 64 bytes (16 words)

    sdio_dat_send(payload, 16);
}


void sdio_send_block(uint32_t * payload) {
    // Standard 512 byte block = 128 words.
    sdio_dat_send(payload, 128);
}


/**
 * Receives exactly one 512-byte block + 64-bit CRC.
 * Monitor both DAT FIFO and CMD FIFO (for Stop Command).
 * @return 0 on success, >0 if a command index was seen, -1 on timeout.
 */
int sdio_receive_block(uint32_t * payload) {
    const int words_per_block = 130; // (512 bytes + 8 bytes CRC) / 4
    int received = 0;
    uint32_t cmd_arg;

    // Timeout for the first bit to arrive (Standard is 250ms for writes)
    uint64_t deadline = time_us_64() + 250000;

    while (received < words_per_block) {
        // While waiting for data, also check for any incoming commands (e.g., CMD12)
        while (pio_sm_is_rx_fifo_empty(sdio_pio, SM_DAT)) {
            int next_cmd = sdio_handle_cmd(&cmd_arg, payload); // Use payload as temp scratch
            if (next_cmd != -1) return next_cmd; // Exit with the command index
            if (time_us_64() > deadline) return -1; // Timeout
        }
        
        // Data is ready, pull it
        payload[received++] = pio_sm_get(sdio_pio, SM_DAT);
        // Reset deadline for subsequent words once the block starts
        deadline = time_us_64() + 50000; 
    }

    return 0; // Success
}

/**
 * SDIO command handler.
 * @return command index if a command was processed, -1 if no command in FIFO.
 */
int sdio_handle_cmd(uint32_t *arg_out, uint32_t *payload) {
    if (pio_sm_is_rx_fifo_empty(sdio_pio, SM_CMD)) return -1;

    // 1. Capture the 48-bit command frame (3 words)
    uint32_t hw1 = pio_sm_get_blocking(sdio_pio, SM_CMD);
    uint8_t cmd_num = (hw1 >> 9) & 0x3F;

    // 2. Immediate Response Path
    if (cmd_num == 0) is_app_cmd = false;
    else if (cmd_num == 2) sdio_cmd_respond(sd_resp_cid, 18);
    else if (cmd_num == 3) sdio_cmd_respond(sd_resp_r6, 6);
    else if (cmd_num == 8) sdio_cmd_respond(sd_resp_r7, 6);
    else if (cmd_num == 9) sdio_cmd_respond(sd_resp_csd, 18);
    else if (cmd_num == 41 && is_app_cmd) sdio_cmd_respond(sd_resp_r3, 6);
    else sdio_cmd_respond(sd_resp_r1, 6);
    // CMD6 (Switch Fn), CMD7 (Select), CMD12 (Stop), CMD13 (Status), 
    // CMD17/18 (Read), CMD24/25 (Write), CMD55 (App-prefix), ACMD6, ACMD51

    uint32_t hw2 = pio_sm_get_blocking(sdio_pio, SM_CMD);
    uint32_t hw3 = pio_sm_get_blocking(sdio_pio, SM_CMD);
    uint32_t arg = (hw1 & 0x1FF) << 23 | (hw2 << 7) | (hw3 >> 9);
    if (arg_out) *arg_out = arg;

    // 3. State-dependent Data Phase Triggers
    bool was_app_cmd = is_app_cmd;
    is_app_cmd = (cmd_num == 55);

    if (cmd_num == 51 && was_app_cmd) sdio_scr_response(payload);
    else if (cmd_num == 6 && !was_app_cmd) sdio_swfn_status(payload, arg >> 31);

    return (int)cmd_num;
}
