#include "actuation/motor_driver.h"

#include <algorithm>

#include <esp_err.h>

#include "config.h"

MotorDriver::MotorDriver(const Wiring &wiring)
    : timer_(wiring.timer), in1_(wiring.in1), in2_(wiring.in2)
{
}

void MotorDriver::start()
{
    const ledc_timer_config_t config = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = timer_,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    const esp_err_t configured = ledc_timer_config(&config);
    ESP_ERROR_CHECK(configured);
    in1_.start(timer_);
    in2_.start(timer_);
}

void MotorDriver::apply(float duty)
{
    const float drive = std::clamp(duty, -DUTY_MAX, DUTY_MAX);
    const bool forward = drive >= 0.0f;
    if (forward)
    {
        in1_.hold_high();
        in2_.pulse_low(drive);
    }
    else
    {
        in1_.pulse_low(-drive);
        in2_.hold_high();
    }
}
