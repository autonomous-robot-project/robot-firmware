#pragma once

#include "control/twist.h"
#include "control/wheel_pair.h"

// Inverse kinematics of the differential drive: the speed of each wheel for a twist of the robot,
// v -/+ omega·b/2, with b the track width.
[[nodiscard]] WheelPair inverse_kinematics(const Twist &twist);
