#include "message_conversion.h"

#include <cmath>
#include <cstring>

#include "config.h"

// The controller repeats the fault and reflex codes of State.msg, because it does not depend on
// micro-ROS (ADR 018). The masks are copied bit for bit, so the two must agree.
static_assert(State::FAULT_COGNIZANT == StateMessageFault::COGNIZANT);
static_assert(State::FAULT_DEADLINE_MISSED == StateMessageFault::DEADLINE_MISSED);
static_assert(State::FAULT_WATCHDOG_RESTART == StateMessageFault::WATCHDOG_RESTART);
static_assert(State::FAULT_FRONT_SENSOR == StateMessageFault::FRONT_SENSOR);
static_assert(State::FAULT_TEST == StateMessageFault::TEST);
static_assert(State::REFLEX_FRONT_OBSTACLE == StateMessageReflex::FRONT_OBSTACLE);

static std::optional<Behavior> to_behavior(uint8_t code)
{
    switch (code)
    {
        case CommandMessageBehavior::TWIST:
            return Behavior::TWIST;
        case CommandMessageBehavior::TURN_TO:
            return Behavior::TURN_TO;
        case CommandMessageBehavior::GO_TO:
            return Behavior::GO_TO;
        default:
            return std::nullopt;
    }
}

static uint8_t to_code(Behavior behavior)
{
    switch (behavior)
    {
        case Behavior::TWIST:
            return CommandMessageBehavior::TWIST;
        case Behavior::TURN_TO:
            return CommandMessageBehavior::TURN_TO;
        case Behavior::GO_TO:
            return CommandMessageBehavior::GO_TO;
    }
    return CommandMessageBehavior::TWIST;  // unreachable: the switch covers every value
}

std::optional<Command> to_command(const CommandMessage &message)
{
    const std::optional<Behavior> behavior = to_behavior(message.behavior);
    if (!behavior)
    {
        return std::nullopt;
    }
    return Command{
        .behavior = *behavior,
        .linear = message.linear,
        .angular = message.angular,
        .theta = message.theta,
        .x = message.x,
        .y = message.y,
    };
}

static CommandMessage to_message(const Command &command)
{
    return CommandMessage{
        .behavior = to_code(command.behavior),
        .linear = command.linear,
        .angular = command.angular,
        .theta = command.theta,
        .x = command.x,
        .y = command.y,
    };
}

StateMessage to_message(const State &state)
{
    return StateMessage{
        .faults = state.faults,
        .reflexes = state.reflexes,
        .echo = to_message(state.echo),
    };
}

InstrumentationMessage to_message(const Instrumentation &instrumentation)
{
    return InstrumentationMessage{
        .expiries = instrumentation.expiry.expiries,
        .expiry_delay_us = instrumentation.expiry.expiry_delay_us,
        .max_execution_time_us = instrumentation.deadlines.max_execution_time_us,
        .max_response_time_us = instrumentation.deadlines.max_response_time_us,
        .deadline_misses = instrumentation.deadlines.deadline_misses,
        .test_event_index = instrumentation.sequencer_response.test_event_index,
        .test_event_response_us = instrumentation.sequencer_response.test_event_response_us,
        .left_wheel_speed = instrumentation.wheel_speeds.left,
        .right_wheel_speed = instrumentation.wheel_speeds.right,
    };
}

// A rosidl string over a static C string: serialized by reading, never freed.
static rosidl_runtime_c__String to_string(const char *text)
{
    const size_t length = std::strlen(text);
    return {.data = const_cast<char *>(text), .size = length, .capacity = length + 1};
}

static builtin_interfaces__msg__Time to_time(int64_t stamp_ns)
{
    constexpr int64_t nanoseconds_per_second = 1000000000;
    return {
        .sec = static_cast<int32_t>(stamp_ns / nanoseconds_per_second),
        .nanosec = static_cast<uint32_t>(stamp_ns % nanoseconds_per_second),
    };
}

// The heading as a rotation about z: (0, 0, sin(θ/2), cos(θ/2)).
static geometry_msgs__msg__Quaternion to_quaternion(float heading)
{
    return {
        .x = 0.0,
        .y = 0.0,
        .z = std::sin(heading / 2.0),
        .w = std::cos(heading / 2.0),
    };
}

OdometryMessage to_message(const Odometry &odometry, int64_t stamp_ns)
{
    OdometryMessage message{};
    message.header.stamp = to_time(stamp_ns);
    message.header.frame_id = to_string(ODOMETRY_FRAME);
    message.child_frame_id = to_string(ODOMETRY_CHILD_FRAME);
    message.pose.pose.position.x = odometry.pose.x;
    message.pose.pose.position.y = odometry.pose.y;
    message.pose.pose.orientation = to_quaternion(odometry.pose.theta);
    message.twist.twist.linear.x = odometry.twist.linear;
    message.twist.twist.angular.z = odometry.twist.angular;
    return message;
}
