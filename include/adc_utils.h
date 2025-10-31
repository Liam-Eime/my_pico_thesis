#pragma once

#include <stdint.h>

// Convert a 12-bit ADC sample to a floating-point voltage (assumes v_ref rail)
static inline float adc_to_voltage(uint16_t sample, float v_ref) {
    return (sample * v_ref) / 4095.0f;
}

// Convert an ADC value expressed in raw counts (can be float) to volts
static inline float adc_counts_to_voltage(float counts, float v_ref) {
    return (counts * v_ref) / 4095.0f;
}
