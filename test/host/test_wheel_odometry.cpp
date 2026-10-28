// Unit test of WheelOdometry on the host, with synthetic edge counts. Each new edge must be a
// travel of s in the locked direction of its wheel (ADR 048): equal counts give a straight line,
// opposite directions a turn in place of 2 s / b per edge pair, and the count may wrap.
// Criticality: T1, evidence (robot-docs codigo.md).

#include <cmath>
#include <cstdint>
#include <numbers>

#include "check.h"
#include "control_config.h"
#include "sensing/wheel_odometry.h"

constexpr WheelPair FORWARD{.left = 1.0f, .right = 1.0f};
constexpr WheelPair TURNING_LEFT{.left = -1.0f, .right = 1.0f};
constexpr float TOLERANCE = 1e-4f;

static bool near(float value, float expected)
{
    return std::abs(value - expected) < TOLERANCE;
}

static void test_straight_line()
{
    WheelOdometry odometry;
    odometry.update({.left = 0, .right = 0}, FORWARD);
    odometry.update({.left = 100, .right = 100}, FORWARD);

    const Pose pose = odometry.pose();
    check(near(pose.x, 100 * WHEEL_TRAVEL_PER_EDGE), "100 edges on both wheels: x = 100 s");
    check(near(pose.y, 0.0f) && near(pose.theta, 0.0f), "a straight line keeps y and theta");
}

static void test_backward()
{
    WheelOdometry odometry;
    odometry.update({.left = 10, .right = 10}, {.left = -1.0f, .right = -1.0f});

    check(near(odometry.pose().x, -10 * WHEEL_TRAVEL_PER_EDGE), "backward edges decrease x");
}

static void test_turn_in_place()
{
    WheelOdometry odometry;
    odometry.update({.left = 5, .right = 5}, TURNING_LEFT);

    const Pose pose = odometry.pose();
    const float expected_theta = 2.0f * 5 * WHEEL_TRAVEL_PER_EDGE / TRACK_WIDTH;
    check(near(pose.theta, expected_theta), "opposite edges turn by (r - l) / b");
    check(near(pose.x, 0.0f) && near(pose.y, 0.0f), "a turn in place keeps the position");
}

static void test_quarter_turn_then_forward()
{
    WheelOdometry odometry;
    // Turn left until theta reaches pi/2, one edge pair at a time, then drive forward.
    uint32_t left = 0;
    uint32_t right = 0;
    while (odometry.pose().theta < std::numbers::pi_v<float> / 2.0f)
    {
        left++;
        right++;
        odometry.update({.left = left, .right = right}, TURNING_LEFT);
    }
    const float heading = odometry.pose().theta;
    odometry.update({.left = left + 50, .right = right + 50}, FORWARD);

    const Pose pose = odometry.pose();
    const float travel = 50 * WHEEL_TRAVEL_PER_EDGE;
    check(
        near(pose.x, travel * std::cos(heading)), "forward after a turn follows the heading in x"
    );
    check(
        near(pose.y, travel * std::sin(heading)), "forward after a turn follows the heading in y"
    );
}

static void test_counter_wrap()
{
    WheelOdometry odometry;
    // Reaching the wrap from zero takes ~4e9 edges: with opposite directions they only turn, so
    // the position stays at the origin and keeps the float precision for the 5 edges after the
    // wrap.
    constexpr uint32_t before_wrap = UINT32_MAX - 2;
    odometry.update({.left = before_wrap, .right = before_wrap}, TURNING_LEFT);
    odometry.update({.left = before_wrap + 5, .right = before_wrap + 5}, FORWARD);  // wraps to 2

    const float travel = std::hypot(odometry.pose().x, odometry.pose().y);
    check(near(travel, 5 * WHEEL_TRAVEL_PER_EDGE), "5 edges across the wrap: 5 s");
}

int main()
{
    test_straight_line();
    test_backward();
    test_turn_in_place();
    test_quarter_turn_then_forward();
    test_counter_wrap();
    return verdict();
}
