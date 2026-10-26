// Unit test of WheelTrackingMonitor on the host, as in V-02: TWIST(0.2, 0) through the
// acceleration limits, a wheel stalled for 2 s and released. The monitor must stay quiet while
// the wheel starts and runs, report the stall within t* + T_f, clear after the release, and never
// judge a reference below v_min (ADR 030).

#include <cstdio>

#include "check.h"
#include "control/acceleration_limiter.h"
#include "control_config.h"
#include "supervision/wheel_tracking_monitor.h"
#include "wheel_model.h"

constexpr float V_R = 0.2f;  // speed of the trial (V-02)
constexpr int CYCLES_PER_SECOND = 100;

// Time for a stalled wheel to read below (1 - e_f) v_r (V-02, step 6), plus T_f, plus one cycle
// for the monitor to see it.
constexpr float DETECTION_LATENCY_BOUND = WHEEL_TRAVEL_PER_EDGE / ((1.0f - E_F) * V_R) + T_F +
                                          T_C_SECONDS;

// The V-02 trial on one wheel: what the monitor reported in each phase.
struct Trial
{
    public:
        bool reported_before_stall = false;
        float detection_latency = -1.0f;  // s after the stall; -1 if never reported
        float clear_latency = -1.0f;      // s after the release; -1 if still reported at the end
};

static Trial run_trial(float commanded_speed)
{
    ClosedLoopWheel wheel;
    AccelerationLimiter limiter;
    WheelTrackingMonitor monitor;
    Trial trial;

    constexpr int stall_start = 3 * CYCLES_PER_SECOND;  // after start and steady state
    constexpr int stall_end = stall_start + 2 * CYCLES_PER_SECOND;
    constexpr int trial_end = stall_end + 3 * CYCLES_PER_SECOND;
    for (int cycle = 0; cycle < trial_end; cycle++)
    {
        wheel.set_stalled(stall_start <= cycle && cycle < stall_end);

        const float reference = limiter.limit({.linear = commanded_speed, .angular = 0.0f}).linear;
        const float measured = wheel.step(reference, cycle_instant(cycle));
        const bool lost = monitor.update(reference, measured, cycle_instant(cycle));

        const float seconds_after_stall = (cycle - stall_start) * T_C_SECONDS;
        const float seconds_after_release = (cycle - stall_end) * T_C_SECONDS;
        if (cycle < stall_start && lost)
        {
            trial.reported_before_stall = true;
        }
        if (cycle >= stall_start && lost && trial.detection_latency < 0.0f)
        {
            trial.detection_latency = seconds_after_stall;
        }
        if (cycle >= stall_end && !lost && trial.clear_latency < 0.0f)
        {
            trial.clear_latency = seconds_after_release;
        }
        if (cycle >= stall_end && lost)
        {
            trial.clear_latency = -1.0f;
        }
    }
    return trial;
}

static void test_stall_at_trial_speed()
{
    const Trial trial = run_trial(V_R);
    std::printf(
        "      detection %.3f s (bound %.3f s), cleared %.2f s after the release\n",
        trial.detection_latency,
        DETECTION_LATENCY_BOUND,
        trial.clear_latency
    );

    check(!trial.reported_before_stall, "start and steady state: no failure");
    check(
        trial.detection_latency >= 0.0f && trial.detection_latency <= DETECTION_LATENCY_BOUND,
        "stall: reported within t* + T_f"
    );
    check(
        trial.clear_latency >= 0.0f && trial.clear_latency <= T_F,
        "release: cleared within T_f, with no command"
    );
}

static void test_reference_below_v_min()
{
    const Trial trial = run_trial(V_MIN / 2.0f);

    check(trial.detection_latency < 0.0f, "reference below v_min: a stall is not judged");
}

int main()
{
    test_stall_at_trial_speed();
    test_reference_below_v_min();
    return verdict();
}
