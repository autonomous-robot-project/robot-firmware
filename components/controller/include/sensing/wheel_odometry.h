#pragma once

#include <cstdint>

#include "control/pose.h"
#include "control/wheel_pair.h"

// Edge count of each wheel since boot.
struct EdgeCounts
{
    public:
        uint32_t left = 0;
        uint32_t right = 0;
};

// Odometry from the edge counts of the encoders (ADR 048): each new edge is a travel of s, in the
// direction locked for its wheel (ADR 019). The pose integrates the travels at the midpoint
// heading.
// Criticality: T1, evidence (robot-docs codigo.md).
class WheelOdometry
{
    public:
        // Called once per cycle with the edge count of each wheel since boot and the direction of
        // each wheel, +1 forward and -1 backward.
        void update(const EdgeCounts &counts, const WheelPair &directions);

        [[nodiscard]] Pose pose() const { return pose_; }

    private:
        // Travel of `new_edges` edges, unsigned: s per edge.
        [[nodiscard]] static float travel(uint32_t new_edges);

        // The first update counts from zero, as the encoder counters do at boot.
        EdgeCounts last_counts_;
        Pose pose_;
};
