#pragma once

#include <chrono>
#include <optional>
#include <vector>

// Open-loop duty profile applied to both motors: a slow ramp up and down, for the static map and
// the dead zone, then steps forward and backward, for the time constant and the gain. Each step
// is followed by the brake.
class DutySchedule
{
    public:
        DutySchedule();

        // Duty at `elapsed` since the start, positive forward; empty after the end.
        [[nodiscard]] std::optional<float> duty_at(std::chrono::milliseconds elapsed) const;

    private:
        // A linear change of the duty from `start_duty` to `end_duty`; constant if they are equal.
        struct Segment
        {
            public:
                std::chrono::milliseconds duration;
                float start_duty;
                float end_duty;
        };

        void add_ramp(std::chrono::milliseconds duration, float start_duty, float end_duty);
        void add_hold(std::chrono::milliseconds duration, float duty);
        // The duty held for STEP_DURATION, then the brake for BRAKE_DURATION.
        void add_step(float duty);

        std::vector<Segment> segments_;
};
