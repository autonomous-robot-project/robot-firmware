// Unit test of DeadlineMissFault on the host, with synthetic instants. The code must stay active
// for DEADLINE_FAULT_DURATION after the last miss, at least the 10 States that an ephemeral code
// needs to reach the sequencer (ADR 037), and a new miss must restart the duration (ADR 028).

#include <chrono>

#include "check.h"
#include "control_config.h"
#include "supervision/deadline_miss_fault.h"

using std::chrono::microseconds;

// ADR 037: an ephemeral code lasts at least 10 States, one per control cycle.
static constexpr int STATES_TO_REACH_THE_SEQUENCER = 10;

static void test_inactive_without_miss()
{
    const DeadlineMissFault fault;
    check(!fault.active(microseconds{0}), "no code before any miss");
}

static void test_duration_reaches_the_sequencer()
{
    check(
        DEADLINE_FAULT_DURATION >= STATES_TO_REACH_THE_SEQUENCER * T_C,
        "the code lasts at least 10 States (ADR 037)"
    );
}

static void test_active_for_the_duration()
{
    DeadlineMissFault fault;
    const microseconds miss{1'000'000};
    fault.record_miss(miss);

    check(fault.active(miss), "active right after the miss");
    check(fault.active(miss + DEADLINE_FAULT_DURATION - microseconds{1}), "active until the end");
    check(!fault.active(miss + DEADLINE_FAULT_DURATION), "inactive once the duration is over");
}

static void test_new_miss_restarts_the_duration()
{
    DeadlineMissFault fault;
    const microseconds first{1'000'000};
    const microseconds second = first + DEADLINE_FAULT_DURATION / 2;
    fault.record_miss(first);
    fault.record_miss(second);

    check(fault.active(first + DEADLINE_FAULT_DURATION), "a new miss extends the code");
    check(!fault.active(second + DEADLINE_FAULT_DURATION), "until the duration after it");
}

int main()
{
    test_inactive_without_miss();
    test_duration_reaches_the_sequencer();
    test_active_for_the_duration();
    test_new_miss_restarts_the_duration();
    return verdict();
}
