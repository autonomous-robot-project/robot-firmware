#pragma once

#include <cstdint>

#include <nav_msgs/msg/odometry.h>
#include <robot_interfaces/msg/command.h>
#include <robot_interfaces/msg/instrumentation.h>
#include <robot_interfaces/msg/state.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

// C++ names for the C types and constants that rosidl generates for robot_interfaces and nav_msgs.
// micro-ROS only generates C, so the generated names stay confined to this file.

using CommandMessage = robot_interfaces__msg__Command;
using StateMessage = robot_interfaces__msg__State;
using InstrumentationMessage = robot_interfaces__msg__Instrumentation;
using OdometryMessage = nav_msgs__msg__Odometry;

// Behavior codes of CommandMessage (Command.msg).
struct CommandMessageBehavior
{
    public:
        static constexpr uint8_t TWIST = robot_interfaces__msg__Command__TWIST;
        static constexpr uint8_t TURN_TO = robot_interfaces__msg__Command__TURN_TO;
        static constexpr uint8_t GO_TO = robot_interfaces__msg__Command__GO_TO;
};

// Fault codes of StateMessage::faults (State.msg).
struct StateMessageFault
{
    public:
        static constexpr uint32_t COGNIZANT = robot_interfaces__msg__State__FAULT_COGNIZANT;
        static constexpr uint32_t DEADLINE_MISSED =
            robot_interfaces__msg__State__FAULT_DEADLINE_MISSED;
        static constexpr uint32_t WATCHDOG_RESTART =
            robot_interfaces__msg__State__FAULT_WATCHDOG_RESTART;
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

inline const rosidl_message_type_support_t *odometry_type_support()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
}
