#pragma once

#include "control/twist.h"
#include "control/wheel_pair.h"

// Forward kinematics of the differential drive: the twist of the robot for the speed of each
// wheel, v = (left + right) / 2 and ω = (right - left) / L_rodas, with L_rodas the track width.
// Criticality: T2, product (robot-docs codigo.md).
[[nodiscard]] Twist forward_kinematics(const WheelPair &wheel_speeds);
