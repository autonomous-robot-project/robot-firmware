// Criticality: T2, product (robot-docs codigo.md).
#include "control/forward_kinematics.h"

#include "control_config.h"

Twist forward_kinematics(const WheelPair &wheel_speeds)
{
    return {
        .linear = (wheel_speeds.left + wheel_speeds.right) / 2.0f,
        .angular = (wheel_speeds.right - wheel_speeds.left) / TRACK_WIDTH,
    };
}
