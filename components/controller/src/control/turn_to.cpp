// Criticality: T2, product (robot-docs codigo.md).
#include "control/turn_to.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "control_config.h"

Twist turn_to(float heading, const Pose &pose)
{
    constexpr float FULL_TURN = 2.0f * std::numbers::pi_v<float>;
    // std::remainder by a full turn gives the error in [-pi, pi], the shortest turn.
    const float heading_error = std::remainder(heading - pose.theta, FULL_TURN);
    const float speed = std::min(
        OMEGA_MAX, std::sqrt(2.0f * ALPHA_DEC_PROFILE * std::abs(heading_error))
    );

    return {.linear = 0.0f, .angular = std::copysign(speed, heading_error)};
}
