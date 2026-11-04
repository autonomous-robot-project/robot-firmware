#pragma once

// A point in odom (m): the target of GO_TO (ADR 024).
// Criticality: T2, product (robot-docs codigo.md).
struct Position
{
    public:
        float x = 0.0f;
        float y = 0.0f;
};

// Pose of the robot in odom (ADR 024, 048): position x, y (m) and heading θ (rad). θ
// accumulates over the turns; the behaviours use it only through sin, cos or a normalized error.
struct Pose
{
    public:
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
};
