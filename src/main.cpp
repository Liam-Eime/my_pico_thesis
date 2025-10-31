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
#include "event_logger.h"


int main() {
    stdio_init_all();
    // Allow time for USB CDC to enumerate before starting high-rate tasks
    sleep_ms(1000);
    // Configure SPI pins for SD card
    board_init_sd_spi_pins();
    // Start emitter PWM (20 kHz, 25% duty) on BOARD_EMITTER_GPIO
    emitter_pwm_start_20k_25();
    // Initialize ADC sampler: 100 kHz on ADC0 (GPIO 26)
    // Using buffer size of 5000 samples (~50 ms at 100 kHz)
    static uint16_t buf_a[5000];
    static uint16_t buf_b[5000];
    const uint32_t sample_rate = 100000;  // 100 kHz
    const uint32_t period_us = 1000000u / sample_rate;
    adc_sampler_init(BOARD_ADC_GPIO, sample_rate, buf_a, buf_b, 5000);

    // Initialize event logger (threshold and duration are configurable)
    const float trigger_threshold_v = 0.2; // 0.2 V
    const float event_duration_s    = 0.5f;  // 0.5 second
    EventLogger::init(trigger_threshold_v, event_duration_s, 3.3f, "event");
    // Debug prints aid threshold tuning; typically disabled in production
    EventLogger::set_debug(true);
    // Configure envelope sampling rate (~carrier frequency) and pre/post window
    EventLogger::set_envelope_rate(20000.0f); // envelope Fs ≈ 20 kHz
    EventLogger::set_pre_post(0.5f, 0.5f);    // capture ±0.5 s around trigger

    // Reusable buffer for demodulated envelope
    std::vector<float> envelope;
    envelope.reserve(1024); // heuristic; will grow as needed

    // Main loop: when a buffer is ready, demodulate (with in-function LPF) and let the logger handle triggers
    unsigned long env_index = 0; // running index across buffers
    // Envelope IIR cutoff (Hz) at ADC rate; tune as needed
    const float envelope_fc_hz = 3000.0f;
    while (true) {
        uint16_t* ready_buf = NULL;
        uint32_t count = 0;
        if (adc_sampler_take_ready(&ready_buf, &count)) {
            // Compute amplitude envelope for this buffer (20 kHz carrier, 25% duty)
            envelope.clear();
                demodulate_to_envelope(ready_buf, count, envelope,
                                              static_cast<float>(sample_rate),
                                              20000.0f, 0.25f,
                                              envelope_fc_hz);

            // Let the event logger decide whether to write this buffer
            EventLogger::process(envelope);
        } else {
            tight_loop_contents();
        }
    }
}