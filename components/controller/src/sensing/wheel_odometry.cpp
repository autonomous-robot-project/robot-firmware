// Criticality: T1, evidence (robot-docs codigo.md).
#include "sensing/wheel_odometry.h"

#include <cmath>

#include "control_config.h"

void WheelOdometry::update(const EdgeCounts &counts, const WheelPair &directions)
{
    // Unsigned differences: correct across the wrap of the counters.
    const float left_travel = travel(counts.left - last_counts_.left) * directions.left;
    const float right_travel = travel(counts.right - last_counts_.right) * directions.right;
    last_counts_ = counts;

    const float linear_travel = (left_travel + right_travel) / 2.0f;
    const float heading_change = (right_travel - left_travel) / TRACK_WIDTH;
    const float midpoint_heading = pose_.theta + heading_change / 2.0f;
    pose_.x += linear_travel * std::cos(midpoint_heading);
    pose_.y += linear_travel * std::sin(midpoint_heading);
    pose_.theta += heading_change;
}

float WheelOdometry::travel(uint32_t new_edges)
{
    return static_cast<float>(new_edges) * WHEEL_TRAVEL_PER_EDGE;
}
