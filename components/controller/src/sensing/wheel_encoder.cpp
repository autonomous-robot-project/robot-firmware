#include "sensing/wheel_encoder.h"

#include <esp_err.h>

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
    stable_level_ = gpio_get_level(pin_);

    const esp_err_t added = gpio_isr_handler_add(pin_, &WheelEncoder::on_rising_edge, this);
    ESP_ERROR_CHECK(added);

    const esp_timer_create_args_t sampler = {
        .callback = &WheelEncoder::on_sample,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "encoder",
        .skip_unhandled_events = true,
    };
    const esp_err_t created = esp_timer_create(&sampler, &sampler_);
    ESP_ERROR_CHECK(created);
    const esp_err_t started = esp_timer_start_periodic(sampler_, ENCODER_SAMPLE_PERIOD.count());
    ESP_ERROR_CHECK(started);
}

void WheelEncoder::on_rising_edge(void *encoder)
{
    auto *self = static_cast<WheelEncoder *>(encoder);
    const std::chrono::microseconds now{esp_timer_get_time()};
    portENTER_CRITICAL_ISR(&self->lock_);
    self->last_rise_ = now;
    portEXIT_CRITICAL_ISR(&self->lock_);
}

void WheelEncoder::on_sample(void *encoder)
{
    static_cast<WheelEncoder *>(encoder)->sample();
}

void WheelEncoder::sample()
{
    const int level = gpio_get_level(pin_);
    if (level == stable_level_)
    {
        samples_away_ = 0;
        return;
    }

    samples_away_++;
    if (samples_away_ < ENCODER_STABLE_SAMPLES)
    {
        return;
    }

    stable_level_ = level;
    samples_away_ = 0;
    const bool slot = level == 1;
    if (slot)
    {
        portENTER_CRITICAL(&lock_);
        const std::chrono::microseconds last_rise = last_rise_;
        portEXIT_CRITICAL(&lock_);
        // Without a new rising edge (the interrupt missed it), the start of the stable run.
        const bool rise_seen = last_rise > recorded_.last;
        const std::chrono::microseconds stable_run_start =
            std::chrono::microseconds{esp_timer_get_time()} -
            ENCODER_STABLE_SAMPLES * ENCODER_SAMPLE_PERIOD;
        record_slot(rise_seen ? last_rise : stable_run_start);
    }
}

void WheelEncoder::record_slot(std::chrono::microseconds instant)
{
    recorded_.previous = recorded_.last;
    recorded_.last = instant;
    recorded_.count++;

    edges_.write(recorded_);
}
