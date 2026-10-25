#pragma once

#include <cstdint>

// Measurements of the controller for the verifications (ADR 028).
// Counters accumulate since boot.
struct Instrumentation
{
    public:
        uint32_t expiries = 0;         // command expiries since boot
        uint32_t expiry_delay_us = 0;  // last expiry: reception -> control cycle that expired it
};
