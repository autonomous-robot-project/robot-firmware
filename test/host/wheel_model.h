#pragma once

// Models of one wheel for the host tests: the motor as the gains are designed for, the
// single-channel encoder, and the closed loop as on the robot. The motor constants are
// provisional, so the numbers of the tests are too.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "control/wheel_pi.h"
#include "control_config.h"
#include "sensing/encoder_edges.h"
#include "sensing/wheel_speed.h"

// The motor as the gains are designed for: the exact step of the first-order plant over one
// control cycle, with the duty held (zero-order hold).
// A stalled wheel does not move, whatever the duty.
class MotorModel
{
    public:
        [[nodiscard]] float speed() const { return speed_; }

        void set_stalled(bool stalled) { stalled_ = stalled; }

        void apply(float duty)
        {
            speed_ = stalled_ ? 0.0f : MOTOR_POLE * speed_ + MOTOR_STEP_GAIN * duty;
        }

    private:
        float speed_ = 0.0f;
        bool stalled_ = false;
};

// Single-channel encoder on the motor model: one edge every WHEEL_TRAVEL_PER_EDGE of travel, at
// the instant interpolated within the cycle, as the interrupt would stamp it.
class EncoderModel
{
    public:
        void advance(float start_speed, float end_speed, std::chrono::microseconds cycle_start)
        {
            const float mean_speed = std::abs(start_speed + end_speed) / 2.0f;
            float travel_left = mean_speed * T_C_SECONDS;
            float elapsed = 0.0f;
            while (travel_left >= WHEEL_TRAVEL_PER_EDGE - travel_since_edge_)
            {
                const float to_edge = WHEEL_TRAVEL_PER_EDGE - travel_since_edge_;
                elapsed += to_edge / mean_speed;
                travel_left -= to_edge;
                travel_since_edge_ = 0.0f;
                record_edge(
                    cycle_start + std::chrono::microseconds{static_cast<int64_t>(elapsed * 1e6f)}
                );
            }
            travel_since_edge_ += travel_left;
        }

        [[nodiscard]] std::optional<EncoderEdges> edges() const { return edges_; }

    private:
        void record_edge(std::chrono::microseconds instant)
        {
            EncoderEdges next = edges_.value_or(EncoderEdges{});
            next.previous = next.last;
            next.last = instant;
            next.count++;
            edges_ = next;
        }

        float travel_since_edge_ = 0.0f;
        std::optional<EncoderEdges> edges_;
};

// One wheel as on the robot: the PI drives the motor, and the speed reaches the PI through the
// encoder and WheelSpeed.
class ClosedLoopWheel
{
    public:
        // One control cycle at `now` toward `reference`; returns the measured speed of the cycle.
        float step(float reference, std::chrono::microseconds now)
        {
            const float measured = speed_estimate_.update(encoder_.edges(), now, applied_duty_);
            applied_duty_ = pi_.update(reference, measured);

            const float start_speed = motor_.speed();
            motor_.apply(applied_duty_);
            encoder_.advance(start_speed, motor_.speed(), now);

            return measured;
        }

        [[nodiscard]] float true_speed() const { return motor_.speed(); }

        void set_stalled(bool stalled) { motor_.set_stalled(stalled); }

    private:
        WheelPI pi_;
        MotorModel motor_;
        EncoderModel encoder_;
        WheelSpeed speed_estimate_;
        float applied_duty_ = 0.0f;
};

// Instant of a control cycle.
inline std::chrono::microseconds cycle_instant(int cycle)
{
    return cycle * std::chrono::microseconds{T_C};
}
