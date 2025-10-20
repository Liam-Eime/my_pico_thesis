#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "adc_sampler.h"
#include <stdio.h>

// Simple ADC sampler using a repeating timer.
// Double buffering is used to reduce latency while processing/writing samples.


typedef struct {
    uint16_t* buf_a;
    uint16_t* buf_b;
    uint32_t  capacity;   // Samples per buffer
    volatile bool a_ready;
    volatile bool b_ready;
    volatile uint32_t idx;
    bool use_a; // true: writing to A; false: writing to B
} adc_ring_t;

static adc_ring_t g_ring;
static repeating_timer_t g_timer;

static bool adc_timer_cb(repeating_timer_t* t) {
    (void)t;
    uint16_t v = adc_read();
    if (g_ring.use_a) {
        g_ring.buf_a[g_ring.idx++] = v;
        if (g_ring.idx >= g_ring.capacity) {
            g_ring.a_ready = true;
            g_ring.idx = 0;
            g_ring.use_a = false;
        }
    } else {
        g_ring.buf_b[g_ring.idx++] = v;
        if (g_ring.idx >= g_ring.capacity) {
            g_ring.b_ready = true;
            g_ring.idx = 0;
            g_ring.use_a = true;
        }
    }
    return true;
}

void adc_sampler_init(uint gpio_adc, uint32_t sample_rate_hz,
                      uint16_t* buf_a, uint16_t* buf_b, uint32_t capacity) {
    // Map GPIO to ADC input (GPIO26->ADC0, 27->ADC1, 28->ADC2)
    adc_init();
    adc_gpio_init(gpio_adc);
    uint adc_input = gpio_adc - 26; // 26->0, 27->1, 28->2
    adc_select_input(adc_input);

    g_ring.buf_a = buf_a;
    g_ring.buf_b = buf_b;
    g_ring.capacity = capacity;
    g_ring.a_ready = false;
    g_ring.b_ready = false;
    g_ring.idx = 0;
    g_ring.use_a = true;

    // Configure repeating timer for the requested sample rate (period in microseconds)
    int64_t period_us = 1000000 / (int32_t)sample_rate_hz; // e.g., 10 kHz -> 100 µs
    add_repeating_timer_us(-period_us, adc_timer_cb, NULL, &g_timer);
}

bool adc_sampler_take_ready(uint16_t** out_buf, uint32_t* out_count) {
    if (g_ring.a_ready) {
        g_ring.a_ready = false;
        *out_buf = g_ring.buf_a;
        *out_count = g_ring.capacity;
        return true;
    }
    if (g_ring.b_ready) {
        g_ring.b_ready = false;
        *out_buf = g_ring.buf_b;
        *out_count = g_ring.capacity;
        return true;
    }
    return false;
}

void adc_sampler_stop() {
    cancel_repeating_timer(&g_timer);
}
