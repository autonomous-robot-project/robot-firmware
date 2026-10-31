// Unit test of DeadlineMeter on the host, with synthetic instants. The meter must take C as
// end - start and R as end - release, keep the largest of each, and count a cycle as a deadline
// miss only when it ends after release + T_C (V-05). The late cycle is the case that must fail:
// a meter that never counts misses would pass every run of V-05.
// Criticality: T1, evidence (robot-docs codigo.md).

#include <chrono>
#include <cstdint>

#include "check.h"
#include "control_config.h"
#include "instrumentation/deadline_meter.h"

using std::chrono::microseconds;

static const uint32_t T_C_US = static_cast<uint32_t>(microseconds{T_C}.count());

static void test_cycle_on_time()
{
    DeadlineMeter meter;
    const bool missed = meter.record(microseconds{0}, microseconds{50}, microseconds{300});

    const DeadlineMeter::Measurements m = meter.measurements();
    check(m.max_execution_time_us == 250, "C is end - start");
    check(m.max_response_time_us == 300, "R is end - release, so it includes the start delay");
    check(m.deadline_misses == 0, "a cycle that ends before release + T_C is not a miss");
    check(!missed, "and is not reported as one");
}

static void test_cycle_ending_on_the_deadline()
{
    DeadlineMeter meter;
    const bool missed = meter.record(microseconds{0}, microseconds{0}, T_C);

    check(meter.measurements().deadline_misses == 0, "ending exactly at release + T_C meets it");
    check(!missed, "and is not reported as a miss");
}

static void test_late_cycle()
{
    DeadlineMeter meter;
    const microseconds release{1'000'000};
    const bool missed = meter.record(
        release, release + microseconds{10}, release + T_C + microseconds{1}
    );

    const DeadlineMeter::Measurements m = meter.measurements();
    check(m.deadline_misses == 1, "a cycle that ends 1 us after release + T_C is a miss");
    check(missed, "and is reported as one, for the deadline-missed fault");
    check(m.max_response_time_us == T_C_US + 1, "R of the late cycle exceeds T_C");
}

static void test_maxima_and_count_accumulate()
{
    DeadlineMeter meter;
    microseconds release{0};
    (void)meter.record(release, release + microseconds{2'000}, release + T_C + microseconds{500});
    release += T_C;
    (void)meter.record(release, release + microseconds{10}, release + microseconds{100});
    release += T_C;
    (void)meter.record(release, release, release + T_C + microseconds{1});

    const DeadlineMeter::Measurements m = meter.measurements();
    check(m.max_execution_time_us == T_C_US + 1, "C_max is the largest execution time");
    check(m.max_response_time_us == T_C_US + 500, "R_max is the largest response time");
    check(m.deadline_misses == 2, "every late cycle is counted, not only the first");
}

int main()
{
    test_cycle_on_time();
    test_cycle_ending_on_the_deadline();
    test_late_cycle();
    test_maxima_and_count_accumulate();
    return verdict();
}
