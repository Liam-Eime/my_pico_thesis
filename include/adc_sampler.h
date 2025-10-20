#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h" // for uint

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize simple ADC sampling at a fixed rate using a repeating timer.
 *
 * Samples are written into two alternating buffers (double buffering).
 *
 * @param gpio_adc          GPIO number corresponding to ADC input (26->ADC0, 27->ADC1, 28->ADC2).
 * @param sample_rate_hz    Sample rate in Hz (e.g., 10000 for 10 kHz).
 * @param buf_a             Pointer to first sample buffer (capacity elements).
 * @param buf_b             Pointer to second sample buffer (capacity elements).
 * @param capacity          Number of samples per buffer.
 */
void adc_sampler_init(uint gpio_adc, uint32_t sample_rate_hz,
                      uint16_t* buf_a, uint16_t* buf_b, uint32_t capacity);

/**
 * @brief Retrieve a buffer of newly sampled data, if available.
 *
 * If a buffer has been filled by the sampler, this function returns true and
 * sets out_buf to the buffer pointer and out_count to the number of samples.
 * The caller should process the data promptly; ownership remains with the sampler.
 *
 * @param out_buf   Output: pointer to the ready buffer.
 * @param out_count Output: number of samples in the ready buffer.
 * @return true if a buffer is ready; false otherwise.
 */
bool adc_sampler_take_ready(uint16_t** out_buf, uint32_t* out_count);

/**
 * @brief Stop sampling and cancel the repeating timer.
 */
void adc_sampler_stop(void);

#ifdef __cplusplus
}
#endif
