// Criticality: T1, evidence (robot-docs codigo.md).
#include "instrumentation/sequencer_response_meter.h"

#include "control_config.h"

void SequencerResponseMeter::record(
    std::chrono::microseconds release,
    bool requested,
    std::optional<std::chrono::microseconds> stop_arrival,
    uint32_t random_word
)
{
    // An active event ends normally even if the request was withdrawn (ADR 049).
    if (event_start_)
    {
        end_event_if_answered(*event_start_, stop_arrival, random_word);
        return;
    }

    if (!requested)
    {
        next_event_.reset();
        return;
    }
    start_event_when_due(release, random_word);
}

void SequencerResponseMeter::end_event_if_answered(
    std::chrono::microseconds event_start,
    std::optional<std::chrono::microseconds> stop_arrival,
    uint32_t random_word
)
{
    // A TWIST(0, 0) that arrived before the event is not a response to it.
    const bool answered = stop_arrival && *stop_arrival > event_start;
    if (!answered)
    {
        return;
    }
    measurements_.test_event_index++;
    measurements_.test_event_response_us = static_cast<uint32_t>(
        (*stop_arrival - event_start).count()
    );
    event_start_.reset();
    next_event_ = *stop_arrival + interval(random_word);
}

void SequencerResponseMeter::start_event_when_due(
    std::chrono::microseconds release, uint32_t random_word
)
{
    if (!next_event_)
    {
        next_event_ = release + interval(random_word);
        return;
    }
    const bool due = release >= *next_event_;
    if (due)
    {
        event_start_ = release;
        next_event_.reset();
    }
}

std::chrono::microseconds SequencerResponseMeter::interval(uint32_t random_word)
{
    // The bias of the modulo is below 0.03 %, irrelevant to the phase independence of V-07.
    const auto span_us = static_cast<uint32_t>(
        std::chrono::microseconds{T_EV_MAX - T_EV_MIN}.count()
    );
    return T_EV_MIN + std::chrono::microseconds{random_word % (span_us + 1)};
}
