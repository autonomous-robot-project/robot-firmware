#pragma once

#include <cstdint>

#include "interface/command.h"

// Snapshot of the controller at the end of a control cycle (ADR 025).
// Conditions are current state, not events: the sequencer detects the edges (ADR 037).
struct State
{
    public:
        // Fault codes, one bit each in `faults`, with the values of the constants of State.msg
        // (the link checks them at compile time).
        static constexpr uint32_t FAULT_COGNIZANT = 1u << 0;         // RF-02
        static constexpr uint32_t FAULT_DEADLINE_MISSED = 1u << 1;   // ADR 028
        static constexpr uint32_t FAULT_WATCHDOG_RESTART = 1u << 2;  // ADR 029
        static constexpr uint32_t FAULT_FRONT_SENSOR = 1u << 3;      // ADR 021

        // Reflex codes, one bit each in `reflexes`, likewise.
        static constexpr uint32_t REFLEX_FRONT_OBSTACLE = 1u << 0;  // ADR 021

        uint32_t faults = 0;    // active faults, one bit per code
        uint32_t reflexes = 0;  // active reflexes, one bit per code
        Command echo;           // command in force this cycle
};
