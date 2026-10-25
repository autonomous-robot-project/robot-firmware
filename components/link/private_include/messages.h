#pragma once

#include <cstdint>

#include <robot_interfaces/msg/command.h>
#include <robot_interfaces/msg/instrumentation.h>
#include <robot_interfaces/msg/state.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

// C++ names for the C types and constants that rosidl generates for robot_interfaces.
// micro-ROS only generates C, so the generated names stay confined to this file.

using CommandMessage = robot_interfaces__msg__Command;
using StateMessage = robot_interfaces__msg__State;
using InstrumentationMessage = robot_interfaces__msg__Instrumentation;

// Behavior codes of CommandMessage (Command.msg).
struct CommandMessageBehavior
{
    public:
        static constexpr uint8_t TWIST = robot_interfaces__msg__Command__TWIST;
        static constexpr uint8_t TURN_TO = robot_interfaces__msg__Command__TURN_TO;
        static constexpr uint8_t GO_TO = robot_interfaces__msg__Command__GO_TO;
};

inline const rosidl_message_type_support_t *command_type_support()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(robot_interfaces, msg, Command);
}

inline const rosidl_message_type_support_t *state_type_support()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(robot_interfaces, msg, State);
}

inline const rosidl_message_type_support_t *instrumentation_type_support()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(robot_interfaces, msg, Instrumentation);
}
