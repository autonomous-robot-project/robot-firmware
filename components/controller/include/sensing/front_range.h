#pragma once

#include <chrono>
#include <cstdint>

// Latest ranging of the front sensor (ADR 020): the distance, the raw device status, signal rate
// and effective SPAD count of the ST API, and the instant of the measurement, in the esp_timer
// clock. The control task classifies it and judges its age (ADR 021).
struct FrontRange
{
    public:
        uint16_t distance_mm = 0;
        uint8_t device_status = 0;
        uint16_t signal_rate_q9_7 = 0;
        uint16_t effective_spads_q8_8 = 0;
        std::chrono::microseconds measured_at{0};

        // Signal rate per effective SPAD (MCPS), from the fixed-point fields; zero without SPADs.
        [[nodiscard]] float signal_rate_per_spad() const
        {
            constexpr float signal_rate_scale = 1.0f / (1 << 7);  // Q9.7
            constexpr float spads_scale = 1.0f / (1 << 8);        // Q8.8
            if (effective_spads_q8_8 == 0)
            {
                return 0.0f;
            }
            const float signal_rate = static_cast<float>(signal_rate_q9_7) * signal_rate_scale;
            const float spads = static_cast<float>(effective_spads_q8_8) * spads_scale;
            return signal_rate / spads;
        }
};
