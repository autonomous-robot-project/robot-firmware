#include "actuation/driver_input.h"

#include <cmath>

#include <esp_err.h>

#include "config.h"

void DriverInput::start(ledc_timer_t timer)
{
    constexpr uint32_t always_low = 0;
    const ledc_channel_config_t config = {
        .gpio_num = wiring_.pin,
        .speed_mode = PWM_SPEED_MODE,
        .channel = wiring_.channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = always_low,
        .hpoint = 0,
        .flags = {},
    };
    const esp_err_t configured = ledc_channel_config(&config);
    ESP_ERROR_CHECK(configured);
}

void DriverInput::hold_high()
{
    set_high_fraction(1.0f);
}

void DriverInput::pulse_low(float fraction)
{
    set_high_fraction(1.0f - fraction);
}

void DriverInput::set_high_fraction(float fraction)
{
    // The LEDC duty counts the high part of the period: 0 is always low, 2^bits always high.
    constexpr uint32_t always_high = 1u << PWM_RESOLUTION;
    const auto duty = static_cast<uint32_t>(std::lround(fraction * always_high));

    const esp_err_t set = ledc_set_duty(PWM_SPEED_MODE, wiring_.channel, duty);
    ESP_ERROR_CHECK(set);
    // The new duty starts with the next PWM period, so the output never glitches.
    const esp_err_t updated = ledc_update_duty(PWM_SPEED_MODE, wiring_.channel);
    ESP_ERROR_CHECK(updated);
}
