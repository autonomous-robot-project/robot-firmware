#pragma once

// One value for each wheel: a speed in m/s or a duty, with the sign of the forward direction.
struct WheelPair
{
    public:
        float left = 0.0f;
        float right = 0.0f;
};
