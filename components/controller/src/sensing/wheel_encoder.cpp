#include "sensing/wheel_encoder.h"

#include <esp_err.h>
#include <esp_timer.h>

#include "config.h"

void WheelEncoder::start()
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ull << pin_,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    const esp_err_t configured = gpio_config(&config);
    ESP_ERROR_CHECK(configured);

    const esp_err_t added = gpio_isr_handler_add(pin_, &WheelEncoder::on_rising_edge, this);
    ESP_ERROR_CHECK(added);
}

void WheelEncoder::on_rising_edge(void *encoder)
{
    const std::chrono::microseconds now{esp_timer_get_time()};
    static_cast<WheelEncoder *>(encoder)->record_edge(now);
}

void WheelEncoder::record_edge(std::chrono::microseconds now)
{
    const bool bounce = recorded_.count > 0 && now - recorded_.last < ENCODER_MIN_EDGE_INTERVAL;
    if (bounce)
    {
        return;
    }

    recorded_.previous = recorded_.last;
    recorded_.last = now;
    recorded_.count++;

    edges_.write_from_isr(recorded_);
}
