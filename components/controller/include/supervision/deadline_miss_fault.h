#pragma once

#include <chrono>
#include <optional>

// Holds the deadline-missed fault code (ADR 028): active for DEADLINE_FAULT_DURATION after the
// last missed deadline, so that the sequencer sees it even though the miss lasts one cycle
// (ADR 037). Every new miss restarts the duration.
class DeadlineMissFault
{
    public:
        // Called with the end of a control cycle that missed its deadline.
        void record_miss(std::chrono::microseconds end);

        // True while `now` is within DEADLINE_FAULT_DURATION of the last miss.
        [[nodiscard]] bool active(std::chrono::microseconds now) const;

    private:
        std::optional<std::chrono::microseconds> last_miss_;
};
