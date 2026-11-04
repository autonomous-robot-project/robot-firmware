#pragma once

#include <cstdint>

// The three behaviours of the controller (ADR 024).
enum class Behavior : uint8_t
{
    TWIST,
    TURN_TO,
    GO_TO,
};

// Behaviour selected by the sequencer and its parameters (ADR 024).
// Only the fields of the active behaviour are used; the others are zero.
struct Command
{
    public:
        Behavior behavior = Behavior::TWIST;
        float linear = 0.0f;   // TWIST: v (m/s)
        float angular = 0.0f;  // TWIST: ω (rad/s)
        float theta = 0.0f;    // TURN_TO: heading in odom (rad)
        float x = 0.0f;        // GO_TO: target x in odom (m)
        float y = 0.0f;        // GO_TO: target y in odom (m)

        // TWIST(0, 0): the command in force after expiry (RNF-02).
        static constexpr Command stop() { return Command{}; }

        // True for TWIST(0, 0), whatever the unused fields hold.
        [[nodiscard]] constexpr bool is_stop() const
        {
            return behavior == Behavior::TWIST && linear == 0.0f && angular == 0.0f;
        }
};
