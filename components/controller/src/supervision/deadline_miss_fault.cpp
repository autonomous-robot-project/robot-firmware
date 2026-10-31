#include "supervision/deadline_miss_fault.h"

#include "control_config.h"

void DeadlineMissFault::record_miss(std::chrono::microseconds end)
{
    last_miss_ = end;
}

bool DeadlineMissFault::active(std::chrono::microseconds now) const
{
    return last_miss_ && now < *last_miss_ + DEADLINE_FAULT_DURATION;
}
