// Unit test of WheelPI on the host, with synthetic data. The conditional integration (ADR 023)
// must keep the integral term within DUTY_MAX plus one integration step, whatever the reference
// and the wheel do; below v_min the PI brakes and resets the integral (ADR 030).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "control/wheel_pi.h"
#include "control_config.h"

// First-order model of the motor, the one the gains are designed for.
class MotorModel
{
    public:
        [[nodiscard]] float speed() const { return speed_; }

        void apply(float duty)
        {
            const float steady_speed = MOTOR_GAIN * duty;
            speed_ += (steady_speed - speed_) * T_C_SECONDS / MOTOR_TIME_CONSTANT;
        }

    private:
        float speed_ = 0.0f;
};

static int failures = 0;

static void check(bool condition, const char *description)
{
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", description);
    if (!condition)
    {
        failures++;
    }
}

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

// The gains promise a first-order response that settles (2 %) in WHEEL_SETTLING_TIME. The
// discrete loop acts on the speed of the previous cycle, so a few cycles more are accepted.
static void check_settling_time(float start, float reference, const char *description)
{
    constexpr float discrete_loop_margin = 1.1f;
    const float measured = settling_time(start, reference);
    std::printf(
        "      settling time %.0f ms (design %.0f ms)\n",
        measured * 1e3f,
        WHEEL_SETTLING_TIME * 1e3f
    );
    check(measured <= discrete_loop_margin * WHEEL_SETTLING_TIME, description);
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

int main()
{
    test_stalled_wheel();
    test_unreachable_reference();
    test_settling_time();
    test_reference_below_v_min();
    std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
    return failures == 0 ? 0 : 1;
}
