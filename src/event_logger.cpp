#include "event_logger.h"
#include "sd_helpers.h"
#include "adc_utils.h"
#include "pico/stdlib.h"  // time functions
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace EventLogger {

static float g_threshold_v = 0.05f; // default 50 mV
static float g_duration_s = 1.0f;   // default 1 second
static float g_vref = 3.3f;         // ADC reference volts
static char  g_base[32] = "event"; // base filename

static bool g_capturing = false;
static absolute_time_t g_capture_end;
static unsigned long g_event_index = 0; // event file index
static unsigned long g_env_index = 0;   // running envelope sample index

static FIL g_fil; // active file when capturing

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
}

void set_threshold(float threshold_voltage) { g_threshold_v = threshold_voltage; }
void set_duration(float event_duration_seconds) { g_duration_s = event_duration_seconds; }

void process(const std::vector<float>& envelope) {
    if (envelope.empty()) return;

    // Buffer to stage CSV lines before a single f_write
    static char csv_buf[64 * 1024]; // 64 KB staging buffer
    size_t off = 0;

    auto flush_write = [&](void) {
        if (!g_capturing || off == 0) return;
        UINT bw = 0;
        FRESULT fr = f_write(&g_fil, csv_buf, (UINT)off, &bw);
        if (fr != FR_OK || bw != off) {
            printf("event_logger: f_write error: %s (%d), bw=%u of %u\n",
                   FRESULT_str(fr), fr, (unsigned)bw, (unsigned)off);
            // Abort capture on error
            f_close(&g_fil);
            f_unmount("");
            g_capturing = false;
        }
        off = 0;
    };

    absolute_time_t now = get_absolute_time();

    // Helper to start a new event capture
    auto start_capture = [&]() {
        char filename[48];
        make_next_filename(filename, sizeof(filename));
        if (!sd_mount_and_open(&g_fil, filename)) {
            printf("event_logger: failed to open %s\n", filename);
            g_capturing = false;
            return;
        }
        // header
        f_printf(&g_fil, "n,envelope_V\n");
        g_capturing = true;
        g_capture_end = make_timeout_time_ms((uint32_t)(g_duration_s * 1000.0f));
        off = 0;
    };

    // Emit one envelope sample into csv_buf
    auto emit_sample = [&](long n_idx, float env_count) {
        float v = adc_to_voltage((uint16_t)lroundf(fabsf(env_count)), g_vref);
        int n = snprintf(&csv_buf[off], (off < sizeof(csv_buf) ? (sizeof(csv_buf) - off) : 0),
                         "%ld,%.6f\n", n_idx, (double)v);
        if (n > 0) off += (size_t)n;
        if (off >= sizeof(csv_buf)) flush_write();
    };

    // If currently capturing, write all samples and stop when time elapsed
    if (g_capturing) {
        for (size_t i = 0; i < envelope.size(); ++i) {
            emit_sample((long)g_env_index++, envelope[i]);
        }
        if (absolute_time_diff_us(now, g_capture_end) <= 0) {
            // Time reached or passed; flush and close
            flush_write();
            f_sync(&g_fil);
            sd_close_and_unmount(&g_fil);
            g_capturing = false;
        } else {
            flush_write();
        }
        return;
    }

    // Not currently capturing: look for first threshold crossing and start
    for (size_t i = 0; i < envelope.size(); ++i) {
        float v = adc_to_voltage((uint16_t)lroundf(fabsf(envelope[i])), g_vref);
        if (v >= g_threshold_v) {
            start_capture();
            if (!g_capturing) return; // failed to start
            // write from this crossing onward
            for (size_t j = i; j < envelope.size(); ++j) {
                emit_sample((long)g_env_index++, envelope[j]);
            }
            flush_write();
            break;
        }
    }
}

}
