// src/main.cpp
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    while (true) {
        printf("Hello, Pico!\n");
        sleep_ms(1000);
    }
}