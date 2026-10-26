#pragma once

// Planar twist of the robot in base_link: linear speed v (m/s) and angular speed omega (rad/s).
struct Twist
{
    public:
        float linear = 0.0f;
        float angular = 0.0f;
};
