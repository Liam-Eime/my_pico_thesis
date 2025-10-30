#pragma once

#include <stdint.h>

// Convert a 12-bit ADC sample to a floating-point voltage (assumes v_ref rail)
static inline float adc_to_voltage(uint16_t sample, float v_ref) {
    return (sample * v_ref) / 4095.0f;
}
