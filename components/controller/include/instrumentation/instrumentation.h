#pragma once

#include <cstdint>

// Measurements of the controller for the verifications (ADR 028).
// Counters accumulate since boot; the wheel speeds are those of the latest cycle.
struct Instrumentation
{
    public:
        uint32_t expiries = 0;         // command expiries since boot
        uint32_t expiry_delay_us = 0;  // last expiry: reception -> control cycle that expired it
        float left_wheel_speed = 0.0f;   // m/s, positive forward; zero below v_min (V-02)
        float right_wheel_speed = 0.0f;  // m/s, positive forward; zero below v_min (V-02)
};
