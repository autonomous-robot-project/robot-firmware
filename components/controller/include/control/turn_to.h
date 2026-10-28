#pragma once

#include "control/pose.h"
#include "control/twist.h"

// TURN_TO (ADR 024): turns in place towards `heading` (rad, odom), with the convergence profile of
// ADR 030, omega = sign(e_theta) min(omega_max, sqrt(2 alpha_dec |e_theta|)). The heading error
// e_theta is normalized to (-pi, pi], so the command holds in any turn of the odometry heading.
// Criticality: T2, product (robot-docs codigo.md).
[[nodiscard]] Twist turn_to(float heading, const Pose &pose);
