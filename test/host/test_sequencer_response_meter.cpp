// Unit test of SequencerResponseMeter on the host, with synthetic instants. While requested, the
// meter must activate the test code every T_ev, counted from the end of the previous event, and
// end it only at a TWIST(0, 0) that arrived after the activation; the response is that arrival -
// the t_k of the activating cycle (ADR 049, V-07). The cases that must fail are a TWIST(0, 0)
// that arrived before the event, which would give a response without a reaction, and no answer
// at all, which must stop the events instead of producing a small response.
// Criticality: T1, evidence (robot-docs codigo.md).

#include <chrono>
#include <cstdint>
#include <optional>

#include "check.h"
#include "control_config.h"
#include "instrumentation/sequencer_response_meter.h"

using std::chrono::microseconds;

static constexpr uint32_t SHORTEST = 0;  // random word that draws T_EV_MIN
static constexpr bool REQUESTED = true;
static constexpr std::optional<microseconds> NO_STOP = std::nullopt;

// Runs one cycle per T_C from `from` while `until` is not reached; returns the next release.
static microseconds run_cycles(
    SequencerResponseMeter &meter,
    microseconds from,
    microseconds until,
    bool requested,
    std::optional<microseconds> stop_arrival = NO_STOP
)
{
    microseconds release = from;
    while (release < until)
    {
        meter.record(release, requested, stop_arrival, SHORTEST);
        release += T_C;
    }
    return release;
}

// Requests from t = 0 and runs until the first event starts, at T_EV_MIN; returns that release.
static microseconds start_first_event(SequencerResponseMeter &meter)
{
    return run_cycles(meter, microseconds{0}, T_EV_MIN + T_C, REQUESTED) - T_C;
}

static void test_inert_without_request()
{
    SequencerResponseMeter meter;
    run_cycles(meter, microseconds{0}, 10 * T_EV_MAX, !REQUESTED);

    check(!meter.event_active(), "without the request no test event starts");
    check(meter.measurements().test_event_index == 0, "and none is counted");
}

static void test_first_event_after_t_ev()
{
    SequencerResponseMeter meter;
    run_cycles(meter, microseconds{0}, T_EV_MIN, REQUESTED);
    check(!meter.event_active(), "no event before T_ev from the request");

    meter.record(T_EV_MIN, REQUESTED, NO_STOP, SHORTEST);
    check(meter.event_active(), "the event starts at the first release T_ev after the request");
}

static void test_response_is_arrival_minus_release()
{
    SequencerResponseMeter meter;
    const microseconds event_start = start_first_event(meter);
    const microseconds arrival = event_start + microseconds{23'456};

    meter.record(event_start + T_C, REQUESTED, NO_STOP, SHORTEST);
    check(meter.event_active(), "the event stays active until a TWIST(0, 0) arrives");

    meter.record(event_start + 3 * T_C, REQUESTED, arrival, SHORTEST);
    const SequencerResponseMeter::Measurements m = meter.measurements();
    check(!meter.event_active(), "a TWIST(0, 0) that arrived after the activation ends the event");
    check(m.test_event_index == 1, "the answered event is counted");
    check(m.test_event_response_us == 23'456, "the response is the arrival - the activating t_k");
}

static void test_earlier_stop_is_not_a_response()
{
    SequencerResponseMeter meter;
    const microseconds event_start = start_first_event(meter);
    const microseconds republished = event_start - microseconds{1};

    run_cycles(meter, event_start + T_C, event_start + T_EV_MAX, REQUESTED, republished);
    check(meter.event_active(), "a TWIST(0, 0) that arrived before the event does not end it");
    check(meter.measurements().test_event_index == 0, "and gives no response");
}

static void test_no_answer_stops_the_events()
{
    SequencerResponseMeter meter;
    const microseconds event_start = start_first_event(meter);

    run_cycles(meter, event_start + T_C, event_start + 10 * T_EV_MAX, REQUESTED);
    check(meter.event_active(), "without an answer the code stays active");
    check(meter.measurements().test_event_index == 0, "and the index stops");
}

static void test_next_event_counted_from_the_answer()
{
    SequencerResponseMeter meter;
    const microseconds event_start = start_first_event(meter);
    const microseconds arrival = event_start + microseconds{40'000};
    const microseconds answer_cycle = event_start + 5 * T_C;
    meter.record(answer_cycle, REQUESTED, arrival, SHORTEST);

    const microseconds next_due = arrival + T_EV_MIN;
    run_cycles(meter, answer_cycle + T_C, next_due, REQUESTED, arrival);
    check(!meter.event_active(), "no new event before T_ev from the arrival of the answer");

    run_cycles(meter, next_due, next_due + T_C, REQUESTED, arrival);
    check(meter.event_active(), "the next event starts T_ev after the arrival of the answer");
}

static void test_withdrawn_request_ends_normally()
{
    SequencerResponseMeter meter;
    const microseconds event_start = start_first_event(meter);

    meter.record(event_start + T_C, !REQUESTED, NO_STOP, SHORTEST);
    check(meter.event_active(), "withdrawing the request does not end the active event");

    const microseconds arrival = event_start + microseconds{15'000};
    meter.record(event_start + 2 * T_C, !REQUESTED, arrival, SHORTEST);
    check(meter.measurements().test_event_index == 1, "the active event is answered normally");

    run_cycles(meter, event_start + 3 * T_C, event_start + 10 * T_EV_MAX, !REQUESTED, arrival);
    check(!meter.event_active(), "and no event starts after it");
}

static void test_interval_limits()
{
    const auto span_us = static_cast<uint32_t>(microseconds{T_EV_MAX - T_EV_MIN}.count());

    SequencerResponseMeter longest;
    longest.record(microseconds{0}, REQUESTED, NO_STOP, span_us);
    longest.record(T_EV_MAX - microseconds{1}, REQUESTED, NO_STOP, SHORTEST);
    check(!longest.event_active(), "the largest draw is not shorter than T_EV_MAX");
    longest.record(T_EV_MAX, REQUESTED, NO_STOP, SHORTEST);
    check(longest.event_active(), "and not longer");

    SequencerResponseMeter wrapped;
    wrapped.record(microseconds{0}, REQUESTED, NO_STOP, span_us + 1);
    wrapped.record(T_EV_MIN, REQUESTED, NO_STOP, SHORTEST);
    check(wrapped.event_active(), "a draw beyond the span wraps to T_EV_MIN");
}

int main()
{
    test_inert_without_request();
    test_first_event_after_t_ev();
    test_response_is_arrival_minus_release();
    test_earlier_stop_is_not_a_response();
    test_no_answer_stops_the_events();
    test_next_event_counted_from_the_answer();
    test_withdrawn_request_ends_normally();
    test_interval_limits();
    return verdict();
}
