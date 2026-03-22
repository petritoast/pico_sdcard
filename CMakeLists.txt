# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_sdcard C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(pico_sdcard
    pico_sdcard.c
    src/board_config.c
    src/sdio.c
)

pico_set_program_name(pico_sdcard "pico_sdcard")
pico_set_program_version(pico_sdcard "0.1")

# Binary Type: no_flash (runs from RAM)
pico_set_binary_type(pico_sdcard no_flash)
#pico_set_binary_type(pico_sdcard copy_to_ram)

# Generate PIO headers
pico_generate_pio_header(pico_sdcard ${CMAKE_CURRENT_LIST_DIR}/src/pio/sd_clk_strobe.pio)
pico_generate_pio_header(pico_sdcard ${CMAKE_CURRENT_LIST_DIR}/src/pio/sd_cmd.pio)
pico_generate_pio_header(pico_sdcard ${CMAKE_CURRENT_LIST_DIR}/src/pio/sd_dat_tx.pio)
pico_generate_pio_header(pico_sdcard ${CMAKE_CURRENT_LIST_DIR}/src/pio/sd_dat_rx.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_sdcard 0)
pico_enable_stdio_usb(pico_sdcard 1)

# Link libraries
target_link_libraries(pico_sdcard
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
)

target_compile_definitions(pico_sdcard PRIVATE
    PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK=1
)

# Include directories
target_include_directories(pico_sdcard PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(pico_sdcard)
