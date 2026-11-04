// Criticality: T2, product (robot-docs codigo.md).
#include "control/go_to.h"

#include <algorithm>
#include <cmath>

#include "control_config.h"

Twist go_to(const Position &target, const Pose &pose)
{
    const float dx = target.x - pose.x;
    const float dy = target.y - pose.y;
    const float distance_error = std::hypot(dx, dy);
    const float heading_error = std::atan2(dy, dx) - pose.theta;

    const float approach_speed = std::min(V_MAX, std::sqrt(2.0f * A_DEC_PROFILE * distance_error));
    const float linear = approach_speed * std::max(0.0f, std::cos(heading_error));

    // The weight takes ω to zero at the target, where e_θ is undefined.
    const float distance_weight = std::min(1.0f, distance_error / D_0);
    const float steering = K_OMEGA * std::sin(heading_error) * distance_weight;
    const float angular = std::clamp(steering, -OMEGA_MAX, OMEGA_MAX);

    return {.linear = linear, .angular = angular};
}
