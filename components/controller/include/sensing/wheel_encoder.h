#pragma once

#include <chrono>
#include <optional>

#include <driver/gpio.h>

#include "channels/latest_value_channel.h"
#include "sensing/encoder_edges.h"

// Single-channel optical encoder of one wheel. An interrupt on each rising edge records its
// instant and count in the edge channel (ADR 017). Edges closer than ENCODER_MIN_EDGE_INTERVAL are
// bounces of the comparator and are discarded (V-02, step 9).
class WheelEncoder
{
    public:
        explicit WheelEncoder(gpio_num_t pin) : pin_(pin) {}
        WheelEncoder(const WheelEncoder &) = delete;
        WheelEncoder &operator=(const WheelEncoder &) = delete;

        // Configures the pin and its interrupt. Called once, at boot, from core 0 and after the
        // GPIO interrupt service is installed, so that the interrupt runs on core 0.
        void start();

        // Consumer side, called by the control task. Empty before the first edge.
        [[nodiscard]] std::optional<EncoderEdges> latest_edges() const { return edges_.read(); }

    private:
        static void on_rising_edge(void *encoder);

        void record_edge(std::chrono::microseconds now);

        gpio_num_t pin_;
        EncoderEdges recorded_;  // written only by the interrupt
        LatestValueChannel<EncoderEdges> edges_;
};
