#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "sensing/front_range.h"

// What the front sensor says about the path ahead (ADR 021).
enum class FrontCondition : uint8_t
{
    CLEAR,     // a valid ranging at d_stop or beyond, or no target
    OBSTACLE,  // a valid ranging below d_stop
    FAILURE,   // only invalid rangings, or none, for more than T_S
};

// Classifies the latest ranging of the front sensor every control cycle (ADR 021, robot-docs nota
// 0015). Each ranging is a valid measurement, no target, or invalid. An invalid ranging leaves the
// condition of the last usable one in force until that one is older than T_S; from then on, and
// before the first usable ranging, the condition is FAILURE.
// Criticality: T2, product (robot-docs codigo.md).
class FrontRangeClassifier
{
    public:
        [[nodiscard]] FrontCondition classify(
            const std::optional<FrontRange> &latest, std::chrono::microseconds now
        );

    private:
        struct UsableRanging
        {
            public:
                FrontCondition condition;
                std::chrono::microseconds measured_at;
        };

        // The last ranging that was not invalid; empty since boot until the first one.
        std::optional<UsableRanging> last_usable_;
};
