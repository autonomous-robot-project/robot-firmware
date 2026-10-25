#pragma once

#include <optional>

#include "interface/command.h"
#include "instrumentation/instrumentation.h"
#include "messages.h"
#include "interface/state.h"

// Conversion between the messages of robot_interfaces and the types of the controller (ADR 018).

// Empty if the behaviour code is unknown: the message is dropped, and with no new command the
// controller expires the previous one (RNF-02).
std::optional<Command> to_command(const CommandMessage &message);

StateMessage to_message(const State &state);

InstrumentationMessage to_message(const Instrumentation &instrumentation);
