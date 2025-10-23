// src/main.cpp
#include <stdio.h>
#include "pico/stdlib.h"
#include "board_pins.h"
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"
#include "adc_sampler.h"
// PWM for emitter drive
#include "hardware/pwm.h"
#include "hardware/clocks.h"

static FATFS g_fs; // Persistent filesystem object while mounted

/**
 * @brief Start hardware PWM on BOARD_EMITTER_GPIO at 20 kHz, 25% duty.
 *
 * Uses PWM TOP=999 for 1000-step resolution and computes clkdiv from the
 * current system clock so frequency stays ~20 kHz even if clk_sys changes.
 */
static void emitter_pwm_start_20k_25(void) {
    // Route the GPIO to PWM function
    gpio_set_function(BOARD_EMITTER_GPIO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BOARD_EMITTER_GPIO);
    uint chan  = pwm_gpio_to_channel(BOARD_EMITTER_GPIO);

    const uint16_t top = 999; // counter 0..999 => 1000 counts per period
    float clkdiv = (float)clock_get_hz(clk_sys) / (20000.0f * (float)(top + 1));

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, top);
    pwm_config_set_clkdiv(&cfg, clkdiv);

    // Initialize and start the slice
    pwm_init(slice, &cfg, true);

    // Set 25% duty: level = 0.25 * (top+1)
    uint16_t level = (uint16_t)(((uint32_t)(top + 1) * 25u) / 100u);
    pwm_set_chan_level(slice, chan, level);
}

/**
 * @brief Mount the filesystem and open a file for append/write.
 *
 * On success, the file handle is valid and the filesystem remains mounted
 * until sd_close_and_unmount is called.
 *
 * @param fil       Output: file handle to open.
 * @param filename  Path to the file to open/create.
 * @return true on success; false on error (an error is printed).
 */
static bool sd_mount_and_open(FIL* fil, const char* filename) {
    FRESULT fr = f_mount(&g_fs, "", 1);
    if (fr != FR_OK) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    fr = f_open(fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK && fr != FR_EXIST) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        f_unmount("");
        return false;
    }
    return true;
}

/**
 * @brief Close an open file and unmount the filesystem.
 *
 * Errors are printed but otherwise ignored.
 *
 * @param fil  File handle previously opened by sd_mount_and_open.
 */
static void sd_close_and_unmount(FIL* fil) {
    if (fil) {
        FRESULT fr = f_close(fil);
        if (fr != FR_OK) {
            printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        }
    }
    f_unmount("");
}

int main() {
    stdio_init_all();
    // Configure SPI pins for SD card
    board_init_sd_spi_pins();
    // Start emitter PWM (20 kHz, 25% duty) on BOARD_EMITTER_GPIO
    emitter_pwm_start_20k_25();
    // Initialize ADC sampler: 10 kHz on floating ADC0 (GPIO 26)
    // Using buffer size of 1000 samples (~100 ms at 10 kHz)
    static uint16_t buf_a[1000];
    static uint16_t buf_b[1000];
    static char     csv_block_buf[32 * 1000]; // 32 bytes/line worst-case; single write per block
    const uint32_t sample_rate = 10000;
    const uint32_t period_us = 1000000u / sample_rate;
    adc_sampler_init(BOARD_ADC_GPIO, sample_rate, buf_a, buf_b, 1000);

    // Open CSV for appending; write header if the file is empty
    FIL fil;
    bool file_ok = sd_mount_and_open(&fil, "data.csv");
    if (file_ok) {
        if (f_size(&fil) == 0) {
            f_printf(&fil, "t_us,adc_raw\n");
        }
    }

    // Main loop: if a buffer is ready, write samples to CSV
    while (true) {
        uint16_t* ready_buf = NULL;
        uint32_t count = 0;
        if (adc_sampler_take_ready(&ready_buf, &count)) {
            if (file_ok) {
                // Build a CSV block with per-sample timestamps in microseconds
                uint64_t t0_us = to_us_since_boot(get_absolute_time());
                size_t off = 0;
                const size_t max_len = sizeof(csv_block_buf);
                for (uint32_t i = 0; i < count; ++i) {
                    uint64_t ts_us = t0_us + (uint64_t)i * (uint64_t)period_us;
                    int n = snprintf(&csv_block_buf[off], (off < max_len ? (max_len - off) : 0),
                                     "%llu,%u\n",
                                     (unsigned long long)ts_us,
                                     (unsigned)ready_buf[i]);
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