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
    // Give USB CDC a moment to enumerate before kicking off high-rate work
    sleep_ms(1000);
    // Configure SPI pins for the SD card
    board_init_sd_spi_pins();
    // Start emitter PWM (20 kHz, 25% duty) on BOARD_EMITTER_GPIO
    emitter_pwm_start_20k_25();
    // ADC sampler: 100 kHz on ADC0 (GPIO 26), 5000-sample ping-pong buffers (~50 ms)
    static uint16_t buf_a[5000];
    static uint16_t buf_b[5000];
    const uint32_t sample_rate = 100000;  // 100 kHz
    const uint32_t period_us = 1000000u / sample_rate;
    adc_sampler_init(BOARD_ADC_GPIO, sample_rate, buf_a, buf_b, 5000);

    // Event logger configuration
    // const float trigger_threshold_v = 0.9f; // 0.9 V for array receiver
    const float trigger_threshold_v = 0.2f; // 0.2 V  for single receiver
    const float event_duration_s    = 0.5f;  // 0.5 second
    EventLogger::init(trigger_threshold_v, event_duration_s, 3.3f, "event");
    // Enable lightweight debug summaries to help tune thresholds
    EventLogger::set_debug(false);
    // Envelope sampling rate (~carrier frequency) and pre/post window
    // Average over M carrier periods; set Fs accordingly.
    const int env_avg_M = 4; // average across 4 periods -> 5 kHz effective rate
    EventLogger::set_envelope_rate(20000.0f / env_avg_M);
    EventLogger::set_pre_post(0.5f, 0.5f);    // capture ±0.5 s around trigger

    // Envelope scratch buffer
    std::vector<float> envelope;
    envelope.reserve(1024); // reserve; grows as needed

    // On each ready buffer: demodulate + filter; logger handles triggers
    unsigned long env_index = 0; // envelope sample index (cross-buffer)
    // Envelope IIR cutoff (Hz) applied at the envelope rate; tune as needed
    const float envelope_fc_hz = 1500.0f;
    // State for cross-buffer M-period averaging
    static float env_acc_sum = 0.0f;
    static int   env_acc_count = 0;
    std::vector<float> envelope_avg;
    envelope_avg.reserve(1024);
    while (true) {
        uint16_t* ready_buf = NULL;
        uint32_t count = 0;
        if (adc_sampler_take_ready(&ready_buf, &count)) {
            // Demodulate 20 kHz, 25% duty; filter at envelope rate
            envelope.clear();
                demodulate_to_envelope(ready_buf, count, envelope,
                                              static_cast<float>(sample_rate),
                                              20000.0f, 0.25f,
                                              envelope_fc_hz);

            // Average non-overlapping M-period blocks; state persists across buffers
            envelope_avg.clear();
            for (size_t i = 0; i < envelope.size(); ++i) {
                env_acc_sum += envelope[i];
                env_acc_count += 1;
                if (env_acc_count >= env_avg_M) {
                    envelope_avg.push_back(env_acc_sum / (float)env_avg_M);
                    env_acc_sum = 0.0f;
                    env_acc_count = 0;
                }
            }

            // Process with EventLogger
            EventLogger::process(envelope_avg);
        } else {
            tight_loop_contents();
        }
    }
}