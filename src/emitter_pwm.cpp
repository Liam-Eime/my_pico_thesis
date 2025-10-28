#include "emitter_pwm.h"
#include "board_pins.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

void emitter_pwm_start_20k_25(void) {
    // Route the GPIO to PWM function
    gpio_set_function(BOARD_EMITTER_GPIO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BOARD_EMITTER_GPIO);
    uint chan  = pwm_gpio_to_channel(BOARD_EMITTER_GPIO);

    const uint16_t top = 999; // counter 0..999 => 1000 counts per period
    float clkdiv = (float)clock_get_hz(clk_sys) / (20000.0f * (float)(top + 1));

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, top);
    pwm_config_set_clkdiv(&cfg, clkdiv);

    // Initialize and start the slice
    pwm_init(slice, &cfg, true);

    // Set 25% duty: level = 0.25 * (top+1)
    uint16_t level = (uint16_t)(((uint32_t)(top + 1) * 25u) / 100u);
    pwm_set_chan_level(slice, chan, level);
}
