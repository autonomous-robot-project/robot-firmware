#include "control/inverse_kinematics.h"

#include "config.h"

WheelPair inverse_kinematics(const Twist &twist)
{
    const float rotation = twist.angular * TRACK_WIDTH / 2.0f;

    return {.left = twist.linear - rotation, .right = twist.linear + rotation};
}
