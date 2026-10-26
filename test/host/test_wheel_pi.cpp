// Unit test of WheelPI on the host, with synthetic data from a model of the motor and of the
// encoder. The conditional integration (ADR 023) must keep the integral term within DUTY_MAX plus
// one integration step, whatever the reference and the wheel do; the discrete design must settle
// within t_s; the loop must hold the speed down to v_min; below it the PI brakes and resets the
// integral (ADR 030). The motor constants are provisional, so these numbers are too.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "check.h"
#include "control/wheel_pi.h"
#include "control_config.h"
#include "wheel_model.h"

// Largest integral term that the conditional integration allows for errors up to `max_error`.
static float integral_bound(float max_error)
{
    return DUTY_MAX + KI * max_error * T_C_SECONDS;
}

// Runs the PI against the motor model for `cycles` and returns the largest |integral| seen.
static float run_against_model(WheelPI &pi, MotorModel &motor, float reference, int cycles)
{
    float largest_integral = 0.0f;
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        motor.apply(pi.update(reference, motor.speed()));
        largest_integral = std::max(largest_integral, std::abs(pi.integral()));
    }
    return largest_integral;
}

static void test_stalled_wheel()
{
    WheelPI pi;
    constexpr float reference = 0.2f;
    constexpr float stalled = 0.0f;
    constexpr int three_seconds = 300;

    float duty = 0.0f;
    float largest_integral = 0.0f;
    for (int cycle = 0; cycle < three_seconds; cycle++)
    {
        duty = pi.update(reference, stalled);
        largest_integral = std::max(largest_integral, std::abs(pi.integral()));
    }

    check(duty == DUTY_MAX, "stalled wheel: the duty saturates at DUTY_MAX");
    check(largest_integral <= integral_bound(reference), "stalled wheel: the integral is bounded");
}

static void test_unreachable_reference()
{
    WheelPI pi;
    MotorModel motor;
    constexpr float top_speed = MOTOR_GAIN * DUTY_MAX;  // 0.67 m/s
    constexpr float reference = 0.8f;
    static_assert(reference > top_speed);
    constexpr int three_seconds = 300;

    const float largest_integral = run_against_model(pi, motor, reference, three_seconds);

    check(
        largest_integral <= integral_bound(reference),
        "reference above the top speed: the integral is bounded"
    );
}

// Time after which the speed stays within 2 % of the change, starting from `start` (settled).
static float settling_time(float start, float reference)
{
    WheelPI pi;
    MotorModel motor;
    constexpr int long_enough_to_settle = 200;
    run_against_model(pi, motor, start, long_enough_to_settle);

    const float band = 0.02f * std::abs(reference - start);
    int last_cycle_outside_band = 0;
    for (int cycle = 1; cycle <= long_enough_to_settle; cycle++)
    {
        motor.apply(pi.update(reference, motor.speed()));
        if (std::abs(motor.speed() - reference) > band)
        {
            last_cycle_outside_band = cycle;
        }
    }
    return static_cast<float>(last_cycle_outside_band + 1) * T_C_SECONDS;
}

// The discrete design promises a response that settles (2 %) within WHEEL_SETTLING_TIME.
static void check_settling_time(float start, float reference, const char *description)
{
    const float measured = settling_time(start, reference);
    std::printf(
        "      settling time %.0f ms (design %.0f ms)\n",
        measured * 1e3f,
        WHEEL_SETTLING_TIME * 1e3f
    );
    check(measured <= WHEEL_SETTLING_TIME, description);
}

static void test_settling_time()
{
    check_settling_time(0.0f, 0.5f, "step 0 -> 0.5 m/s: settles within t_s");
    check_settling_time(0.4f, -0.4f, "reversal +0.4 -> -0.4 m/s: settles within t_s");
}

static void test_reference_below_v_min()
{
    WheelPI pi;
    MotorModel motor;
    constexpr int one_second = 100;
    run_against_model(pi, motor, 0.2f, one_second);

    const float duty = pi.update(V_MIN / 2.0f, motor.speed());

    check(duty == 0.0f, "reference below v_min: the duty is the brake");
    check(pi.integral() == 0.0f, "reference below v_min: the integral is reset");
}

// The loop measured through the encoder model, as on the robot, at `reference` for 5 s. Returns
// the slowest and the fastest speed of the last 2 s.
static std::pair<float, float> speed_range_with_encoder(float reference)
{
    ClosedLoopWheel wheel;
    float slowest = reference;
    float fastest = reference;

    constexpr int five_seconds = 500;
    constexpr int last_two_seconds = 300;
    for (int cycle = 0; cycle < five_seconds; cycle++)
    {
        wheel.step(reference, cycle_instant(cycle));
        if (cycle >= last_two_seconds)
        {
            slowest = std::min(slowest, wheel.true_speed());
            fastest = std::max(fastest, wheel.true_speed());
        }
    }
    return {slowest, fastest};
}

// At v_min the edges are the sparsest the loop must handle; it must still hold the speed.
static void test_lowest_speed_with_encoder()
{
    const auto [slowest, fastest] = speed_range_with_encoder(V_MIN);
    std::printf("      at v_min = %.3f m/s: speed %.4f..%.4f m/s\n", V_MIN, slowest, fastest);

    const float tolerance = 0.02f * V_MIN;
    check(
        fastest - V_MIN <= tolerance && V_MIN - slowest <= tolerance,
        "encoder at v_min: the speed holds within 2 %"
    );
}

int main()
{
    test_stalled_wheel();
    test_unreachable_reference();
    test_settling_time();
    test_lowest_speed_with_encoder();
    test_reference_below_v_min();
    return verdict();
}
