#pragma once

#include "control/pose.h"
#include "control/twist.h"

// GO_TO (ADR 024): drives towards `target`, with the convergence profile of ADR 030:
// v = min(v_max, sqrt(2 a_dec e_d)) max(0, cos e_θ) and
// ω = K_ω sin e_θ min(1, e_d / d_0), limited to ω_max. e_d is the distance to the
// target and e_θ the error from the heading to the direction of the target; e_θ enters
// only through sin and cos, so it needs no normalization. Both outputs tend to zero at the target.
// Criticality: T2, product (robot-docs codigo.md).
[[nodiscard]] Twist go_to(const Position &target, const Pose &pose);
