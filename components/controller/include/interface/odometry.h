#pragma once

#include <chrono>

#include "control/pose.h"
#include "control/twist.h"

// Odometry of a control cycle, published by the link (ADRs 025, 048): the pose in odom, the twist
// measured by the encoders, and the local instant of the pose, which the link converts to the
// clock of the agent (ADR 027).
// Criticality: T2, product (robot-docs codigo.md).
struct Odometry
{
    public:
        Pose pose;
        Twist twist;
        std::chrono::microseconds instant{0};  // esp_timer clock
};
