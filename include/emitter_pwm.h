#pragma once

/**
 * Hardware PWM for the emitter GPIO.
 *
 * Starts PWM on BOARD_EMITTER_GPIO at 20 kHz with 25% duty. The pin is
 * defined in board_pins.h.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start PWM on BOARD_EMITTER_GPIO at 20 kHz / 25% duty.
 * Configures the pin, slice, frequency, and duty. Call once at startup.
 */
void emitter_pwm_start_20k_25(void);

#ifdef __cplusplus
}
#endif
