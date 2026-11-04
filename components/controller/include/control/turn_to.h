#pragma once

#include "control/pose.h"
#include "control/twist.h"

// TURN_TO (ADR 024): turns in place towards `heading` (rad, odom), with the convergence profile of
// ADR 030, ω = sign(e_θ) min(ω_max, sqrt(2 α_dec |e_θ|)). The heading error
// e_θ is normalized to (−π, π], so the command holds in any turn of the odometry heading.
// Criticality: T2, product (robot-docs codigo.md).
[[nodiscard]] Twist turn_to(float heading, const Pose &pose);
