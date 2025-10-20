#pragma once

/*
  Board pin definitions for RP2040 + microSD (SPI mode).
  Mapping:
    - CS/SS  : GPIO 5
    - MISO   : GPIO 4
    - MOSI   : GPIO 3
    - SCK    : GPIO 2

  This configuration aligns with RP2040's default SPI0 pins (SCK=2, MOSI=3, MISO=4) with CS on GPIO 5.
*/

#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Which SPI peripheral the SD card is on
#define BOARD_SD_SPI_INST spi0

// GPIO assignments for the SD card SPI
#define BOARD_SD_SCK_GPIO   2u
#define BOARD_SD_MOSI_GPIO  3u
#define BOARD_SD_MISO_GPIO  4u
#define BOARD_SD_CS_GPIO    5u

// Card-detect configuration (set BOARD_SD_USE_CARD_DETECT to 0 if not wired)
#define BOARD_SD_USE_CARD_DETECT      0
#define BOARD_SD_CARD_DETECT_GPIO     0u
#define BOARD_SD_CARD_DETECTED_TRUE   1u

// SPI electrical/config defaults
#define BOARD_SD_SPI_MODE   0u  // Mode 0 is standard for SD over SPI
// Start conservative; you can increase once stable (e.g. 31.25 MHz = 125MHz/4)
#define BOARD_SD_BAUD_HZ    (12 * 1000 * 1000)  // 12 MHz

// Optional: drive strengths for outputs (uncomment to enable tuning)
// #define BOARD_SET_DRIVE_STRENGTH     1
// #define BOARD_MOSI_DRIVE_STRENGTH    GPIO_DRIVE_STRENGTH_8MA
// #define BOARD_SCK_DRIVE_STRENGTH     GPIO_DRIVE_STRENGTH_8MA

/**
 * @brief Configure GPIO muxing for SPI0 and set CS high (deasserted).
 *
 * This function sets SCK/MOSI/MISO pins to the SPI function and initializes
 * the CS pin as a GPIO output driven high.
 */
static inline void board_init_sd_spi_pins(void) {
    gpio_set_function(BOARD_SD_SCK_GPIO,  GPIO_FUNC_SPI);
    gpio_set_function(BOARD_SD_MOSI_GPIO, GPIO_FUNC_SPI);
    gpio_set_function(BOARD_SD_MISO_GPIO, GPIO_FUNC_SPI);

    gpio_init(BOARD_SD_CS_GPIO);
    gpio_set_dir(BOARD_SD_CS_GPIO, GPIO_OUT);
  gpio_put(BOARD_SD_CS_GPIO, 1); // Deassert CS

#ifdef BOARD_SET_DRIVE_STRENGTH
    gpio_set_drive_strength(BOARD_SD_MOSI_GPIO, BOARD_MOSI_DRIVE_STRENGTH);
    gpio_set_drive_strength(BOARD_SD_SCK_GPIO,  BOARD_SCK_DRIVE_STRENGTH);
#endif
}

#ifdef __cplusplus
}
#endif
