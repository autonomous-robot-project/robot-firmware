#pragma once

#include <cstdint>

#include "sensing/wheel_encoder.h"

// Prints a CSV line for every new edge of one wheel encoder: edge,<t_us>,<wheel>,<count>.
// Polled faster than the closest edges (15 ms), so no edge is missed; a gap in the count is
// reported as a comment line.
class EdgeReporter
{
    public:
        EdgeReporter(const WheelEncoder &encoder, char wheel) : encoder_(encoder), wheel_(wheel) {}

        void report_new_edge();

    private:
        const WheelEncoder &encoder_;
        char wheel_;
        uint32_t reported_count_ = 0;
};
