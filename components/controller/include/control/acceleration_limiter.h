#pragma once

#include "control/twist.h"

// Asymmetric ramp on the effective reference (ADR 023): the commanded speed reaches the motors
// gradually, at most A_ACCEL while speeding up and A_DEC (steeper) while slowing down, so the
// torque on the gearbox never comes as a step. The angular limits are 2·a / b, so each wheel stays
// within the linear limits.
class AccelerationLimiter
{
    public:
        // The reference of this cycle: the previous one moved toward `target` within the limits.
        [[nodiscard]] Twist limit(const Twist &target);

    private:
        // Largest change per second of one speed while speeding up and while slowing down.
        struct Limits
        {
            public:
                float acceleration;
                float deceleration;
        };

        // One speed, linear or angular, with its own limits.
        class Axis
        {
            public:
                explicit Axis(const Limits &limits) : limits_(limits) {}

                [[nodiscard]] float limit(float target);

            private:
                Limits limits_;
                float value_ = 0.0f;
        };

        static Axis make_linear_axis();
        static Axis make_angular_axis();

        Axis linear_ = make_linear_axis();
        Axis angular_ = make_angular_axis();
};
