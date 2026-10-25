#pragma once

#include <chrono>
#include <optional>

#include "instrumentation/instrumentation.h"

// Measures the command expiry for V-06 (RNF-02), as part of the instrumentation (ADR 028):
// the time from the reception of a command to the control cycle that expired it.
class ExpiryMeter
{
    public:
        // Called in every control cycle in which the command that arrived at `arrival` is
        // expired; only the first call for each command is an expiry.
        void record(std::chrono::microseconds arrival, std::chrono::microseconds cycle);

        [[nodiscard]] Instrumentation measurements() const { return measurements_; }

    private:
        std::optional<std::chrono::microseconds> last_expired_arrival_;
        Instrumentation measurements_;
};
