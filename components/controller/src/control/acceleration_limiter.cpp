#include "control/acceleration_limiter.h"

#include <algorithm>

#include "config.h"

Twist AccelerationLimiter::limit(const Twist &target)
{
    return {.linear = linear_.limit(target.linear), .angular = angular_.limit(target.angular)};
}

AccelerationLimiter::Axis AccelerationLimiter::make_linear_axis()
{
    return Axis{{.acceleration = A_ACCEL, .deceleration = A_DEC}};
}

AccelerationLimiter::Axis AccelerationLimiter::make_angular_axis()
{
    return Axis{{
        .acceleration = 2.0f * A_ACCEL / TRACK_WIDTH,
        .deceleration = 2.0f * A_DEC / TRACK_WIDTH,
    }};
}

float AccelerationLimiter::Axis::limit(float target)
{
    const float change = target - value_;

    // Away from zero is speeding up; toward zero, or across it, is slowing down.
    const bool speeding_up = value_ * change > 0.0f || (value_ == 0.0f && change != 0.0f);
    const float limit = speeding_up ? limits_.acceleration : limits_.deceleration;
    const float limit_per_cycle = limit * T_C_SECONDS;

    value_ += std::clamp(change, -limit_per_cycle, limit_per_cycle);
    return value_;
}
