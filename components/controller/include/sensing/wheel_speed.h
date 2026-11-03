#pragma once

#include <chrono>
#include <optional>

#include "sensing/encoder_edges.h"

// Speed of one wheel from its encoder edges (V-02): s / dt between the last two edges, bounded by
// s / (time since the last edge), so the estimate decays while no edge arrives. Below v_min the
// measure is not valid and the speed is zero (no edge for T_VEL). The single channel gives no
// direction: it stays locked while the wheel turns and, with the wheel stopped, follows the
// applied duty (ADR 019).
class WheelSpeed
{
    public:
        // Speed in m/s at `now`, positive forward. `applied_duty` is the duty of the previous
        // cycle.
        [[nodiscard]] float update(
            const std::optional<EncoderEdges> &edges,
            std::chrono::microseconds now,
            float applied_duty
        );

        // Direction locked for the wheel (ADR 019): +1 forward, -1 backward.
        [[nodiscard]] float direction() const { return direction_; }

        // The reference the wheel may follow: zero, a brake, while it still turns against the
        // sign of `reference`, so it reverses only from a stop and the direction follows the
        // duty (ADR 019). Otherwise a quick reversal keeps the old direction, the PI pushes the
        // wrong way and the wheel runs away (robot-docs nota 0018).
        [[nodiscard]] float reference_from_stop(float reference) const;

    private:
        [[nodiscard]] static float magnitude(
            const std::optional<EncoderEdges> &edges, std::chrono::microseconds now
        );

        float direction_ = 1.0f;  // +1 forward, -1 backward
        float speed_ = 0.0f;      // of the latest update
};
