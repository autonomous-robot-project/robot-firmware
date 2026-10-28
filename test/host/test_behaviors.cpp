// Unit test of TURN_TO and GO_TO on the host (ADR 024, 030), with synthetic poses. TURN_TO must
// turn the shortest way whatever the turn of the odometry heading, with the sqrt profile limited
// to omega_max; GO_TO must slow down to zero at the target, not advance with the target behind,
// and steer towards the target with a weight that fades out near it.

#include <cmath>
#include <numbers>

#include "check.h"
#include "control/go_to.h"
#include "control/turn_to.h"
#include "control_config.h"

constexpr float PI = std::numbers::pi_v<float>;
constexpr float TOLERANCE = 1e-4f;

static bool near(float value, float expected)
{
    return std::abs(value - expected) < TOLERANCE;
}

static void test_turn_to_profile()
{
    const Twist small = turn_to(0.1f, Pose{});
    check(
        near(small.angular, std::sqrt(2.0f * ALPHA_DEC_PROFILE * 0.1f)), "sqrt profile at 0.1 rad"
    );
    check(small.linear == 0.0f, "TURN_TO turns in place");
    check(near(turn_to(PI / 2.0f, Pose{}).angular, OMEGA_MAX), "large error limited to omega_max");
    check(turn_to(-0.1f, Pose{}).angular < 0.0f, "negative error turns clockwise");
    check(turn_to(0.0f, Pose{}).angular == 0.0f, "no error, no turn");
}

static void test_turn_to_in_any_turn()
{
    // The odometry accumulated 6.5 rad; the target pi/2 ahead given in the first turn.
    const Pose pose{.theta = 6.5f};
    const float target = std::remainder(6.5f, 2.0f * PI) + PI / 2.0f;
    check(turn_to(target, pose).angular > 0.0f, "target ahead in another turn: shortest way");
    const Twist near_target = turn_to(std::remainder(6.5f, 2.0f * PI) + 0.1f, pose);
    check(near(near_target.angular, turn_to(0.1f, Pose{}).angular), "same error, same output");
}

static void test_go_to_far_ahead()
{
    const Twist twist = go_to({.x = 1.0f, .y = 0.0f}, Pose{});
    check(near(twist.linear, V_MAX), "target far ahead: v_max");
    check(near(twist.angular, 0.0f), "target straight ahead: no steering");
}

static void test_go_to_slows_down_at_the_target()
{
    const float distance = 0.01f;
    const Twist twist = go_to({.x = distance, .y = 0.0f}, Pose{});
    check(
        near(twist.linear, std::sqrt(2.0f * A_DEC_PROFILE * distance)),
        "sqrt profile near the target"
    );
    const Twist at_target = go_to({.x = 0.0f, .y = 0.0f}, Pose{});
    check(at_target.linear == 0.0f && at_target.angular == 0.0f, "at the target: (0, 0)");
}

static void test_go_to_target_behind()
{
    const Twist twist = go_to({.x = -1.0f, .y = 0.1f}, Pose{});
    check(twist.linear == 0.0f, "target behind: no forward speed");
    check(twist.angular > 0.0f, "target behind on the left: turns left");
}

static void test_go_to_steering()
{
    const Twist left = go_to({.x = 1.0f, .y = 0.1f}, Pose{});
    const float heading_error = std::atan2(0.1f, 1.0f);
    check(near(left.angular, K_OMEGA * std::sin(heading_error)), "far: K_omega sin e_theta");
    check(
        go_to({.x = 1.0f, .y = -0.1f}, Pose{}).angular < 0.0f, "target on the right: turns right"
    );

    const Twist close = go_to({.x = 0.05f, .y = 0.05f}, Pose{});
    const float weight = std::hypot(0.05f, 0.05f) / D_0;
    check(
        near(close.angular, K_OMEGA * std::sin(PI / 4.0f) * weight), "near: weighted by e_d / d_0"
    );
}

static void test_go_to_in_any_turn()
{
    const Twist first_turn = go_to({.x = 1.0f, .y = 0.1f}, Pose{});
    const Twist third_turn = go_to({.x = 1.0f, .y = 0.1f}, Pose{.theta = 4.0f * PI});
    check(near(first_turn.angular, third_turn.angular), "same pose two turns later: same output");
}

int main()
{
    test_turn_to_profile();
    test_turn_to_in_any_turn();
    test_go_to_far_ahead();
    test_go_to_slows_down_at_the_target();
    test_go_to_target_behind();
    test_go_to_steering();
    test_go_to_in_any_turn();
    return verdict();
}
