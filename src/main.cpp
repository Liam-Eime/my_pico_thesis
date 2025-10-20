// src/main.cpp
#include <stdio.h>
#include "pico/stdlib.h"
#include "board_pins.h"

int main() {
    stdio_init_all();
    // Prepare SPI pins for SD card (safe even if not used yet)
    board_init_sd_spi_pins();
    while (true) {
        printf("Hello, Pico!\n");
        sleep_ms(1000);
    }
}