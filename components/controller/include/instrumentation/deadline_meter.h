#pragma once

#include <chrono>
#include <cstdint>

// Measures the deadlines of the control task for V-05 (RNF-01), as part of the instrumentation
// (ADR 028): the execution and response times of each cycle, and the cycles that ended after
// their deadline, one period T_C after the nominal release.
// Criticality: T1, evidence (robot-docs codigo.md).
class DeadlineMeter
{
    public:
        // Accumulated since boot (ADR 028).
        struct Measurements
        {
            public:
                uint32_t max_execution_time_us = 0;  // C_max: end - start
                uint32_t max_response_time_us = 0;   // R_max: end - nominal release t_k
                uint32_t deadline_misses = 0;        // cycles that ended after t_k + T_C
        };

        // Called once per cycle with its nominal release t_k, its start and its end.
        void record(
            std::chrono::microseconds release,
            std::chrono::microseconds start,
            std::chrono::microseconds end
        );

        [[nodiscard]] Measurements measurements() const { return measurements_; }

    private:
        Measurements measurements_;
};
