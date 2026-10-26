#pragma once

#include <chrono>
#include <optional>

// Watches whether one wheel tracks its reference (RF-02, cognizant failure): the wheel has lost
// the tracking when its speed error stays above e_f·|reference| for T_f. The reference is the
// one after the acceleration limits, so a limited acceleration is not read as a failure
// (ADR 023). Below v_min the speed is not observable and the wheel is not judged (ADR 030).
class WheelTrackingMonitor
{
    public:
        // True while the wheel is not tracking its reference. Clears by itself as soon as the
        // error is back within the limit, with no command (RF-02).
        [[nodiscard]] bool update(float reference, float measured, std::chrono::microseconds now);

    private:
        // When the error last went above the limit; empty while it is within.
        std::optional<std::chrono::microseconds> error_since_;
};
