#include "event_logger.h"
#include "sd_helpers.h"
#include "adc_utils.h"
#include "pico/stdlib.h"  // time functions
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace EventLogger {

static float g_threshold_v = 1.0f; // default 1.0 V
static float g_duration_s = 1.0f;   // default 1 second (used if no pre/post configured)
static float g_vref = 3.3f;         // ADC reference volts
static char  g_base[32] = "event"; // base filename

static bool g_capturing = false;
static bool g_debug = false;
static absolute_time_t g_capture_end; // legacy time-based capture end (if used)
static unsigned long g_event_index = 0; // event file index
static unsigned long g_env_index = 0;   // running envelope sample index

static FIL g_fil; // active file (used only when writing completed event)

// Envelope sampling rate (samples/second) for converting seconds to samples
static float g_env_fs = 20000.0f; // default ~20 kHz (carrier freq)

// Pre/post durations for symmetric capture around the trigger
static float g_pre_s = 0.0f;   // seconds before trigger
static float g_post_s = 1.0f;  // seconds after trigger (default old behavior)

// Derived sample counts
static size_t g_pre_count = 0;   // samples to include before trigger
static size_t g_post_count = 20000; // samples to include after trigger
static size_t g_post_remaining = 0; // countdown while capturing

// Ring buffer to retain the last g_pre_count samples prior to trigger
struct RingSample { long n_idx; float counts; };
static std::vector<RingSample> g_ring;   // fixed-capacity circular buffer
static size_t g_ring_cap = 0;            // capacity (== g_pre_count)
static size_t g_ring_size = 0;           // current number of valid samples
static size_t g_ring_head = 0;           // next write position

// Event buffer: pre + trigger + post window in RAM; written once at end
static std::vector<RingSample> g_event;

static inline void ring_reset() {
    g_ring_size = 0;
    g_ring_head = 0;
}

static inline void ring_configure(size_t cap) {
    g_ring_cap = cap;
    g_ring.clear();
    g_ring.resize(g_ring_cap); // allocate fixed storage
    ring_reset();
}

static inline void ring_push(long n_idx, float counts) {
    if (g_ring_cap == 0) return;
    g_ring[g_ring_head] = { n_idx, counts };
    g_ring_head = (g_ring_head + 1) % g_ring_cap;
    if (g_ring_size < g_ring_cap) g_ring_size++;
}

static inline void recompute_counts_from_times() {
    g_pre_count = (size_t)floorf(g_pre_s * g_env_fs + 0.5f);
    g_post_count = (size_t)floorf(g_post_s * g_env_fs + 0.5f);
    ring_configure(g_pre_count);
}

static void make_next_filename(char* out, size_t out_sz) {
    // event_0001.csv style
    snprintf(out, out_sz, "%s_%04lu.csv", g_base, ++g_event_index);
}

void init(float threshold_voltage, float event_duration_seconds,
          float v_ref, const char* base_filename) {
    g_threshold_v = threshold_voltage;
    g_duration_s = event_duration_seconds;
    g_vref = v_ref;
    if (base_filename && *base_filename) {
        strncpy(g_base, base_filename, sizeof(g_base) - 1);
        g_base[sizeof(g_base) - 1] = '\0';
    }
    g_capturing = false;
    g_env_index = 0;
    // Default to legacy behavior: 0 s pre, duration_s post
    g_pre_s = 0.0f;
    g_post_s = g_duration_s;
    recompute_counts_from_times();
}

void set_threshold(float threshold_voltage) { g_threshold_v = threshold_voltage; }
void set_duration(float event_duration_seconds) {
    g_duration_s = event_duration_seconds;
    // If using legacy duration (no pre/post explicitly set), update post window
    if (g_pre_s == 0.0f) { g_post_s = g_duration_s; recompute_counts_from_times(); }
}
void set_debug(bool enable) { g_debug = enable; }

void set_envelope_rate(float fs_envelope_hz) {
    if (fs_envelope_hz > 0.0f) {
        g_env_fs = fs_envelope_hz;
        recompute_counts_from_times();
    }
}

void set_pre_post(float pre_seconds, float post_seconds) {
    if (pre_seconds < 0) pre_seconds = 0;
    if (post_seconds < 0) post_seconds = 0;
    g_pre_s = pre_seconds;
    g_post_s = post_seconds;
    recompute_counts_from_times();
}

void process(const std::vector<float>& envelope) {
    if (envelope.empty()) return;

    absolute_time_t now = get_absolute_time();

    // Start a new event capture
    auto start_capture = [&]() {
        // Preallocate capacity for the full event window (pre + post)
        g_event.clear();
        g_event.reserve(g_pre_count + g_post_count + 4);
        g_capturing = true;
        // Time kept for debug output
        g_capture_end = make_timeout_time_ms((uint32_t)((g_pre_s + g_post_s) * 1000.0f));
    };

    // Append one envelope sample to the event buffer
    auto buffer_sample = [&](long n_idx, float env_count) {
        g_event.push_back({ n_idx, env_count });
    };

    // Process samples: ring, trigger detection, and capture buffering
    float min_v = 1e30f, max_v = -1e30f;
    for (size_t i = 0; i < envelope.size(); ++i) {
        float counts = envelope[i];
        float v = adc_counts_to_voltage(counts, g_vref);
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;

        if (g_capturing) {
            // Capture mode: buffer to RAM and count down post samples
            buffer_sample((long)g_env_index, counts);
            if (g_post_remaining > 0) {
                g_post_remaining--;
                if (g_post_remaining == 0) {
                    // Finish: write event once to SD
                    char filename[48];
                    make_next_filename(filename, sizeof(filename));
                    if (!sd_mount_and_open(&g_fil, filename)) {
                        printf("event_logger: failed to open %s\n", filename);
                    } else {
                        f_printf(&g_fil, "n,envelope_V\n");
                        for (const auto& rs : g_event) {
                            float v = adc_counts_to_voltage(rs.counts, g_vref);
                            f_printf(&g_fil, "%ld,%.6f\n", rs.n_idx, (double)v);
                        }
                        f_sync(&g_fil);
                        sd_close_and_unmount(&g_fil);
                    }
                    g_event.clear();
                    g_capturing = false;
                    if (g_debug) printf("event_logger: capture complete (n up to %lu)\n", g_env_index);
                }
            }
            // Keep ring/history and sample index going even while capturing
            ring_push((long)g_env_index, counts);
            g_env_index++;
            continue;
        }

        // Not capturing: check for trigger first (pre window should not include current sample)
        if (v <= g_threshold_v) {
            if (g_debug) {
                printf("event_logger: trigger at n=%lu (buf idx=%u), v=%.3fV <= thr=%.3fV; buf min=%.3fV max=%.3fV\n",
                       g_env_index, (unsigned)i, (double)v, (double)g_threshold_v, (double)min_v, (double)max_v);
            }
            start_capture();
            if (!g_capturing) {
                // Failed to open file; still advance index/ring
                ring_push((long)g_env_index, counts);
                g_env_index++;
                continue;
            }

            // Append pre-window from ring (oldest -> newest)
            size_t to_write = g_ring_size < g_pre_count ? g_ring_size : g_pre_count;
            if (to_write > 0) {
                size_t oldest = (g_ring_head + g_ring_cap - g_ring_size) % g_ring_cap;
                for (size_t k = 0; k < to_write; ++k) {
                    size_t idx = (oldest + (g_ring_size - to_write) + k) % g_ring_cap; // last 'to_write' samples
                    buffer_sample(g_ring[idx].n_idx, g_ring[idx].counts);
                }
            }

            // Emit current (trigger) sample and set post countdown
            g_post_remaining = (g_post_count > 0) ? (g_post_count - 1) : 0; // include trigger as first
            buffer_sample((long)g_env_index, counts);

            // Advance ring and index
            ring_push((long)g_env_index, counts);
            g_env_index++;
            continue; // continue buffering post samples
        }

        // No trigger; add to history ring and advance index
        ring_push((long)g_env_index, counts);
        g_env_index++;
    }

    if (!g_capturing && g_debug) {
        // No trigger; print min/max for threshold tuning
        printf("event_logger: idle; buf min=%.3fV max=%.3fV thr=%.3fV\n",
               (double)min_v, (double)max_v, (double)g_threshold_v);
    }
}

}
