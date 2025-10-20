/* hw_config.c (project-specific) */
#include "hw_config.h"
#include "sd_card.h"
#include "SPI/my_spi.h"
#include "board_pins.h"

/* SPI object configured from board_pins.h */
static spi_t spi = {
    .hw_inst = BOARD_SD_SPI_INST,
    .sck_gpio = BOARD_SD_SCK_GPIO,
    .mosi_gpio = BOARD_SD_MOSI_GPIO,
    .miso_gpio = BOARD_SD_MISO_GPIO,
    .baud_rate = BOARD_SD_BAUD_HZ,
    .spi_mode = BOARD_SD_SPI_MODE,
};

/* SPI interface for the SD card */
static sd_spi_if_t spi_if = {
    .spi = &spi,
    .ss_gpio = BOARD_SD_CS_GPIO,
};

/* One SD card on SPI */
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if,
    .use_card_detect = BOARD_SD_USE_CARD_DETECT,
    .card_detect_gpio = BOARD_SD_CARD_DETECT_GPIO,
    .card_detected_true = BOARD_SD_CARD_DETECTED_TRUE,
};

size_t sd_get_num(void) { return 1; }

sd_card_t* sd_get_by_num(size_t num) {
    if (num == 0) return &sd_card;
    return NULL;
}
