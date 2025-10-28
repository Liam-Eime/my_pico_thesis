// src/main.cpp
#include <stdio.h>
#include "pico/stdlib.h"
#include <vector>
#include <cmath>
#include "board_pins.h"
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"
#include "adc_sampler.h"
#include "emitter_pwm.h"
#include "sd_helpers.h"


int main() {
    stdio_init_all();
    // Configure SPI pins for SD card
    board_init_sd_spi_pins();
    // Start emitter PWM (20 kHz, 25% duty) on BOARD_EMITTER_GPIO
    emitter_pwm_start_20k_25();
    // Initialize ADC sampler: 100 kHz on ADC0 (GPIO 26)
    // Using buffer size of 5000 samples (~50 ms at 100 kHz)
    static uint16_t buf_a[5000];
    static uint16_t buf_b[5000];
    static char     csv_block_buf[32 * 5000]; // 32 bytes/line worst-case; single write per block
    const uint32_t sample_rate = 100000;  // 100 kHz
    const uint32_t period_us = 1000000u / sample_rate;
    adc_sampler_init(BOARD_ADC_GPIO, sample_rate, buf_a, buf_b, 5000);

    // Open CSV for appending; write header if the file is empty
    FIL fil;
    bool file_ok = sd_mount_and_open(&fil, "data.csv");
    if (file_ok) {
        if (f_size(&fil) == 0) {
            // Envelope CSV header: index and envelope (ADC LSBs)
            f_printf(&fil, "n,envelope\n");
        }
    }

    // Reusable buffer for demodulated envelope
    std::vector<float> envelope;
    envelope.reserve(1024); // heuristic; will grow as needed

    // Main loop: if a buffer is ready, demodulate and write envelope to CSV
    unsigned long env_index = 0; // running index across buffers
    while (true) {
        uint16_t* ready_buf = NULL;
        uint32_t count = 0;
        if (adc_sampler_take_ready(&ready_buf, &count)) {
            // Compute amplitude envelope for this buffer (20 kHz carrier, 25% duty)
            envelope.clear();
            demodulate_to_envelope(ready_buf, count, envelope,
                                   static_cast<float>(sample_rate),
                                   20000.0f, 0.25f);

            if (file_ok) {
                // Build a CSV block of envelope samples: "n,envelope" per line
                size_t off = 0;
                const size_t max_len = sizeof(csv_block_buf);
                for (size_t i = 0; i < envelope.size(); ++i) {
                    long val = lroundf(envelope[i]);
                    int n = snprintf(&csv_block_buf[off], (off < max_len ? (max_len - off) : 0),
                                     "%lu,%ld\n",
                                     env_index++, val);
                    if (n < 0) { break; }
                    off += (size_t)n;
                    if (off >= max_len) { break; }
                }

                if (off > 0) {
                    UINT bw = 0;
                    FRESULT fr = f_write(&fil, csv_block_buf, (UINT)off, &bw);
                    if (fr != FR_OK || bw != off) {
                        printf("f_write error: %s (%d), bw=%u of %u\n", FRESULT_str(fr), fr, (unsigned)bw, (unsigned)off);
                    }
                    // Flush to reduce data loss risk
                    f_sync(&fil);
                }
            }
        } else {
            tight_loop_contents();
        }
    }
}