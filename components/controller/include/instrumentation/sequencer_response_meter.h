#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

// Generates the test events of V-07 (RNF-03) and measures the response of the sequencer, as part
// of the instrumentation (ADR 049): while requested, the test code is activated every T_ev and
// stays active until a TWIST(0, 0) arrives; the response is that arrival - the release t_k of the
// cycle that activated the code.
// Criticality: T1, evidence (robot-docs codigo.md).
class SequencerResponseMeter
{
    public:
        // Accumulated since boot (ADR 028).
        struct Measurements
        {
            public:
                uint32_t test_event_index = 0;        // test events answered since boot
                uint32_t test_event_response_us = 0;  // last answered event: t_k -> arrival
        };

        // Called once per cycle with its nominal release t_k. `stop_arrival` is the arrival of the
        // latest command if it is TWIST(0, 0); `random_word` draws T_ev when an event is
        // scheduled.
        void record(
            std::chrono::microseconds release,
            bool requested,
            std::optional<std::chrono::microseconds> stop_arrival,
            uint32_t random_word
        );

        // True while the test code is active (State::FAULT_TEST).
        [[nodiscard]] bool event_active() const { return event_start_.has_value(); }

        [[nodiscard]] Measurements measurements() const { return measurements_; }

    private:
        // Ends the active event if a TWIST(0, 0) arrived after it started, and schedules the next
        // one T_ev after that arrival.
        void end_event_if_answered(
            std::optional<std::chrono::microseconds> stop_arrival, uint32_t random_word
        );

        // Schedules the next event, or starts it if its instant has come.
        void start_event_when_due(std::chrono::microseconds release, uint32_t random_word);

        // T_ev in [T_EV_MIN, T_EV_MAX].
        [[nodiscard]] static std::chrono::microseconds interval(uint32_t random_word);

        std::optional<std::chrono::microseconds> event_start_;  // t_k of the activating cycle
        std::optional<std::chrono::microseconds> next_event_;   // scheduled while requested
        Measurements measurements_;
};
