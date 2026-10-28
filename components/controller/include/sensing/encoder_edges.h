#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

// Rising edges of one wheel encoder, as recorded by the interrupt (ADR 017): the count since boot
// and the instants of the last two edges, in the esp_timer clock.
struct EncoderEdges
{
    public:
        uint32_t count = 0;
        std::chrono::microseconds last{0};
        std::chrono::microseconds previous{0};
};

// The edges of both wheels, read once per control cycle; each one is empty before its first slot.
struct WheelEdges
{
    public:
        std::optional<EncoderEdges> left;
        std::optional<EncoderEdges> right;
};
