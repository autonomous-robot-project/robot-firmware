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
};
