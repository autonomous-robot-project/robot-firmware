#include "sensing/wheel_speed.h"

#include <algorithm>

#include "config.h"

float WheelSpeed::update(
    const std::optional<EncoderEdges> &edges, std::chrono::microseconds now, float applied_duty
)
{
    const float speed = magnitude(edges, now);
    const bool stopped = speed == 0.0f;
    const bool driven = applied_duty != 0.0f;
    if (stopped && driven)
    {
        direction_ = applied_duty > 0.0f ? 1.0f : -1.0f;
    }
    return direction_ * speed;
}

float WheelSpeed::magnitude(
    const std::optional<EncoderEdges> &edges, std::chrono::microseconds now
)
{
    const bool no_interval_yet = !edges || edges->count < 2;
    if (no_interval_yet)
    {
        return 0.0f;
    }
    const auto since_last_edge = now - edges->last;
    const auto last_interval = edges->last - edges->previous;
    const auto bound = std::max(last_interval, since_last_edge);
    if (bound >= T_VEL)
    {
        return 0.0f;
    }
    return WHEEL_TRAVEL_PER_EDGE / std::chrono::duration<float>(bound).count();
}
