#pragma once

#include <cstdint>
#include <optional>

#include "interface/command.h"
#include "instrumentation/instrumentation.h"
#include "interface/odometry.h"
#include "messages.h"
#include "interface/state.h"

// Conversion between the messages of robot_interfaces and the types of the controller (ADR 018).

// Empty if the behaviour code is unknown: the message is dropped, and with no new command the
// controller expires the previous one (RNF-02).
std::optional<Command> to_command(const CommandMessage &message);

StateMessage to_message(const State &state);

InstrumentationMessage to_message(const Instrumentation &instrumentation);

// The pose and twist in the odom and base_link frames, stamped at `stamp_ns` in the clock of the
// agent (ADR 027). The frame names point to static strings, which the message never frees.
OdometryMessage to_message(const Odometry &odometry, int64_t stamp_ns);
