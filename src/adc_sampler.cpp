#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "adc_sampler.h"
#include <stdio.h>
#include <vector>


// ADC sampler with synchronous demodulation.
// - Sampling: a repeating timer reads ADC at the requested Fs and writes into
//   a ping-pong (double) buffer. When a buffer fills it is flagged ready and
//   the writer switches buffers. Consumers poll via adc_sampler_take_ready(...).
// - Demodulation: demodulate_to_envelope() performs square-wave coherent
//   demodulation given carrier_freq and duty. It multiplies samples by +1
//   during the ON portion and -1 during the OFF portion of each carrier period,
//   averages over one full period, and outputs one envelope sample per period
//   (envelope rate ≈ Fc). A trailing partial period is discarded.
// Notes:
//   - ADC samples are 12-bit unsigned (0..4095) in uint16_t.
//   - No DC removal, scaling, or filtering is applied.
//   - Timer-based sampling (no DMA); suitable for kHz-range sample rates.


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

void demodulate_to_envelope(const uint16_t* in_buf,
                            size_t count,
                            std::vector<float>& out_env,
                            float sample_rate,
                            float carrier_freq,
                            float duty)
{
    // Samples per carrier period ≈ Fs / Fc
    int samples_per_period = static_cast<int>(sample_rate / carrier_freq + 0.5f);
    if (samples_per_period <= 0) return;
    // Number of ON samples in each period
    int on_samples  = static_cast<int>(samples_per_period * duty + 0.5f);
    if (on_samples <= 0) on_samples = 1;
    if (on_samples >= samples_per_period) on_samples = samples_per_period - 1;

    // Accumulate ON and OFF separately within each period, then output (avg_on - avg_off)
    float acc_on = 0.0f, acc_off = 0.0f;
    int cnt_on = 0, cnt_off = 0;

    for (size_t i = 0; i < count; ++i) {
        int phase = static_cast<int>(i % samples_per_period);
        float s = static_cast<float>(in_buf[i]);
        if (phase < on_samples) { acc_on += s; cnt_on++; }
        else { acc_off += s; cnt_off++; }

        // End of period: emit one envelope sample (signed, counts)
        if (phase == samples_per_period - 1) {
            float avg_on = (cnt_on > 0) ? (acc_on / cnt_on) : 0.0f;
            float avg_off = (cnt_off > 0) ? (acc_off / cnt_off) : 0.0f;
            out_env.push_back(avg_on - avg_off);
            acc_on = acc_off = 0.0f;
            cnt_on = cnt_off = 0;
        }
    }
}

void adc_sampler_stop() {
    cancel_repeating_timer(&g_timer);
}
