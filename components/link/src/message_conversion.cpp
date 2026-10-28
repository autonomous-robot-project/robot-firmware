#include "message_conversion.h"

// The controller repeats the fault codes of State.msg, because it does not depend on micro-ROS
// (ADR 018). The faults mask is copied bit for bit, so the two must agree.
static_assert(State::FAULT_COGNIZANT == StateMessageFault::COGNIZANT);
static_assert(State::FAULT_DEADLINE_MISSED == StateMessageFault::DEADLINE_MISSED);
static_assert(State::FAULT_WATCHDOG_RESTART == StateMessageFault::WATCHDOG_RESTART);

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
        .left_wheel_speed = instrumentation.wheel_speeds.left,
        .right_wheel_speed = instrumentation.wheel_speeds.right,
    };
}
