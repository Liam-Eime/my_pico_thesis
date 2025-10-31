#pragma once

#include <vector>
#include <stdint.h>

namespace EventLogger {

// Initialize the event logger with a voltage threshold and capture duration.
// v_ref is the ADC reference voltage used for counts->volts conversion.
void init(float threshold_voltage, float event_duration_seconds,
          float v_ref = 3.3f, const char* base_filename = "event");

// Optionally adjust parameters at runtime
void set_threshold(float threshold_voltage);
void set_duration(float event_duration_seconds);
// Enable/disable lightweight debug prints (min/max per buffer, triggers)
void set_debug(bool enable);

// Set the envelope sampling rate (Hz). Required for pre/post windows.
void set_envelope_rate(float fs_envelope_hz);

// Configure a pre/post capture window around the trigger.
// For example, pre=0.5f and post=0.5f captures ±0.5 s (1 s total).
void set_pre_post(float pre_seconds, float post_seconds);

// Process one demodulated envelope buffer; logs events to SD when triggered
void process(const std::vector<float>& envelope);

}
