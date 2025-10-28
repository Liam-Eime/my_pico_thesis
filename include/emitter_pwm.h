#pragma once

/**
 * @file emitter_pwm.h
 * @brief Helper to drive the emitter GPIO with hardware PWM.
 *
 * Provides a convenience initializer to start PWM on BOARD_EMITTER_GPIO
 * at 20 kHz with 25% duty cycle. The emitter pin is defined in board_pins.h.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start PWM on BOARD_EMITTER_GPIO at 20 kHz and 25% duty.
 *
 * Configures the pin function, PWM slice, frequency and duty.
 * Safe to call once at startup; PWM runs continuously thereafter.
 */
void emitter_pwm_start_20k_25(void);

#ifdef __cplusplus
}
#endif
