#include "message_conversion.h"

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
