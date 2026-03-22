# Pico SD Card Emulator / USB HS Link

SD Card Emulator for Pico 2 (RP2350). By emulating a standard SD card in 4-bit mode, this allows one to use cheap USB card reader ICs for streaming arbitrary data over a block device interface. With 6 GPIOs, speeds of ~12.5MB/s or ~25MB/s (or even ~40MB/s in boost mode) are possible. Can (be modified to) work with any SDIO host controller, not just USB.

## Hardware
 SDIO was forced/limited to 3.3V signalling, to keep the setup simple. The project is geared towards ICs like GL823K (Genesys Logic) that cost <$0.40 in single qty. This code was tested with a generic USB3.0 card reader, with long soldered wires and set to the default speed mode (25 MHz SD_CLK). Pico 2 is clocked at 250 MHz. The card reader was connected to the Pico 2 as follows:

```
SD_CMD  <-> GP0
SD_CLK   -> GP1
SD_DAT0 <-> GP2
SD_DAT1 <-> GP3
SD_DAT2 <-> GP4
SD_DAT3 <-> GP5
```

## Challenges
The SDIO protocol is not very well documented, and the information available is scattered across various sources or paywalled. Not to mention the timing constraints of the protocol (also not available in the public datasheets), which are (needlessly?) strict for command and token responses. The basic goal was to quickly bring the host to a data transfer state. The "canned" responses used here might not satisfy all SD card readers. CRC16 calculation could limit max throughput.

## Host Side Interface
Tested on a Windows 11 host, the emulated card appears as a 128 MB FAT16 volume. The MBR (LBA 0) and boot sector (LBA 1) were copied from RP2350's own UF2 bootloader "virtual" volume. I expected aggressive read-aheads or caching, but it works 1:1 when the volume is opened with the right flags. 512-byte aligned reads/writes are translated to exactly that on SDIO side. USBPcap is also useful here for monitoring SCSI/BOT.

## Pico 2 Implementation

*   **PIO State Machines**: Three SMs (in a single PIO block) handle the timing-critical bit-shuffling:
    *   `SM_CLK_STROBE`: Rising/falling edge detection/IRQs.
    *   `SM_CMD`: Command frame capture and synchronous response.
    *   `SM_DAT`: Data transmission/reception (swapped dynamically).

* **Back-pressure** for multi block_reads is simple enough, just don't send the block (very long timeouts). For block_writes, need to set DAT0 low to give us time to process the data (not implemented yet).

* **Fast CRC** implementation based on [SDIO_RP2350](https://github.com/rabbitholecomputing/SDIO_RP2350). Internal DMA sniffer based CRC-16 isn't useful here as the CRC is applied to each DAT line. De-interleaving the bits to accomdate it is an option, though unlikely to improve performance much.

* **Dual Cores** were used here, just because stdout-over-USB was interrupting the timely handling of CMD responses (leading to timeouts/resets). So using the second core isn't strictly necessary, but it's the better option for stability and higher throughput.

## Future Work
* Build and test on a custom PCB with a GL823K IC, needs very few components. Just a couple caps and resistors.
* Test the boost mode as claimed by the IC. Guess they included it to saturate USB HS speeds, without the complications of 1.8V voltage switch.
* Create an external strobe signal from the SD_CLK rising edges, would simplify the PIO SMs and allow for higher clock speeds.
* USB 3.0 card readers too are quite cheap, and SD104 mode would allow for ~90 MB/s but that involves 1.8V signalling (possible) and the need to sample a 104/208 MHz SD_CLK (unlikely). Most resources mention this mode uses dynamic phase alignment, so who knows.
* Find a better MBR/boot sector for the emulated volume? The current one was copied from the UF2 bootloader.
* Check if dynamic CRC response tokens might be feasible, currently returning success always.