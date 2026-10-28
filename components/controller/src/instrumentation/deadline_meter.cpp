// Criticality: T1, evidence (robot-docs codigo.md).
#include "instrumentation/deadline_meter.h"

#include <algorithm>

#include "control_config.h"

void DeadlineMeter::record(
    std::chrono::microseconds release,
    std::chrono::microseconds start,
    std::chrono::microseconds end
)
{
    const auto execution_time_us = static_cast<uint32_t>((end - start).count());
    const auto response_time_us = static_cast<uint32_t>((end - release).count());
    measurements_.max_execution_time_us = std::max(
        measurements_.max_execution_time_us, execution_time_us
    );
    measurements_.max_response_time_us = std::max(
        measurements_.max_response_time_us, response_time_us
    );

    // The deadline is the next release; ending exactly on it still meets it (V-05).
    const bool deadline_missed = end > release + T_C;
    if (deadline_missed)
    {
        measurements_.deadline_misses++;
    }
}
