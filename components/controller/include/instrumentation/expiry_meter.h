#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

// Measures the command expiry for V-06 (RNF-02), as part of the instrumentation (ADR 028):
// the time from the reception of a command to the control cycle that expired it.
class ExpiryMeter
{
    public:
        // Accumulated since boot (ADR 028).
        struct Measurements
        {
            public:
                uint32_t expiries = 0;         // command expiries since boot
                uint32_t expiry_delay_us = 0;  // last expiry: reception -> cycle that expired it
        };

        // Called in every control cycle in which the command that arrived at `arrival` is
        // expired; only the first call for each command is an expiry.
        void record(std::chrono::microseconds arrival, std::chrono::microseconds cycle);

        [[nodiscard]] Measurements measurements() const { return measurements_; }

    private:
        std::optional<std::chrono::microseconds> last_expired_arrival_;
        Measurements measurements_;
};
