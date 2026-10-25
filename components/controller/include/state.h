#pragma once

#include <cstdint>

#include "command.h"

// Snapshot of the controller at the end of a control cycle (ADR 025).
// Conditions are current state, not events: the sequencer detects the edges (ADR 037).
struct State
{
    public:
        uint32_t faults = 0;    // active faults, one bit per code
        uint32_t reflexes = 0;  // active reflexes, one bit per code
        Command echo;           // command in force this cycle
};
