#include "duty_schedule.h"

#include <array>

using std::chrono::milliseconds;

// DUTY_MAX of the controller (ADR 023): the stalled wheel gets at most the rated voltage.
constexpr float DUTY_LIMIT = 0.48f;
constexpr milliseconds RAMP_DURATION{20000};
constexpr milliseconds STEP_DURATION{3000};
constexpr milliseconds BRAKE_DURATION{2000};
constexpr std::array STEP_DUTIES = {0.1f, 0.2f, 0.3f, 0.4f, DUTY_LIMIT};
constexpr float BRAKE = 0.0f;

DutySchedule::DutySchedule()
{
    add_ramp(RAMP_DURATION, BRAKE, DUTY_LIMIT);
    add_ramp(RAMP_DURATION, DUTY_LIMIT, BRAKE);
    add_hold(BRAKE_DURATION, BRAKE);
    for (const float direction : {1.0f, -1.0f})
    {
        for (const float duty : STEP_DUTIES)
        {
            add_step(direction * duty);
        }
    }
}

std::optional<float> DutySchedule::duty_at(milliseconds elapsed) const
{
    milliseconds segment_start{0};
    for (const Segment &segment : segments_)
    {
        const milliseconds into_segment = elapsed - segment_start;
        if (into_segment < segment.duration)
        {
            const float fraction = static_cast<float>(into_segment.count()) /
                                   static_cast<float>(segment.duration.count());
            return segment.start_duty + fraction * (segment.end_duty - segment.start_duty);
        }
        segment_start += segment.duration;
    }
    return std::nullopt;
}

void DutySchedule::add_ramp(milliseconds duration, float start_duty, float end_duty)
{
    segments_.push_back({.duration = duration, .start_duty = start_duty, .end_duty = end_duty});
}

void DutySchedule::add_hold(milliseconds duration, float duty)
{
    add_ramp(duration, duty, duty);
}

void DutySchedule::add_step(float duty)
{
    add_hold(STEP_DURATION, duty);
    add_hold(BRAKE_DURATION, BRAKE);
}
