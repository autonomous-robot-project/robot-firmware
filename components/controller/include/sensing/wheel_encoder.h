#pragma once

#include <chrono>
#include <optional>

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "channels/latest_value_channel.h"
#include "sensing/encoder_edges.h"

// Single-channel optical encoder of one wheel. The comparator of the sensor has no hysteresis, so
// its output chatters at both transitions of a slot (robot-docs nota 0010). A slot counts when
// the sampled output has been high for ENCODER_STABLE_SAMPLES after being stable low. Its instant
// is the last rising edge seen by the interrupt before that, so the chatter adds no edges and the
// instant keeps the precision of the interrupt. Instant and count go to the edge channel (ADR 017).
class WheelEncoder
{
    public:
        explicit WheelEncoder(gpio_num_t pin) : pin_(pin) {}
        WheelEncoder(const WheelEncoder &) = delete;
        WheelEncoder &operator=(const WheelEncoder &) = delete;

        // Configures the pin, its interrupt and the sampling timer. Called once, at boot, from
        // core 0 and after the GPIO interrupt service is installed, so that both run on core 0.
        void start();

        // Consumer side, called by the control task. Empty before the first slot.
        [[nodiscard]] std::optional<EncoderEdges> latest_edges() const { return edges_.read(); }

    private:
        static void on_rising_edge(void *encoder);
        static void on_sample(void *encoder);

        void sample();
        void record_slot(std::chrono::microseconds instant);

        gpio_num_t pin_;
        esp_timer_handle_t sampler_ = nullptr;
        portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
        std::chrono::microseconds last_rise_{0};  // written by the interrupt, under lock_
        int stable_level_ = 0;                    // the rest only by the sampler
        int samples_away_ = 0;  // consecutive samples away from stable_level_
        EncoderEdges recorded_;
        LatestValueChannel<EncoderEdges> edges_;
};
