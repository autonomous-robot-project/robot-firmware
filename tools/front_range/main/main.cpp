// Bench tool: the rangings of the front sensor, for T_S and the status table of ADR 021
// (robot-docs nota 0015). Runs the FrontRangeReader of the robot firmware, with its recovery,
// and prints CSV on the USB console:
//   range,<t_us>,<distance_mm>,<device_status>   every new ranging, t_us its measured_at
// The warnings of the sensor task go to the same console. tools/front_range/front_range.py
// records and summarizes them. Nothing drives the motors.
// Criticality: T1, evidence (robot-docs codigo.md).

#include <chrono>
#include <cstdio>
#include <optional>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sensing/front_range.h"
#include "sensing/front_range_reader.h"

using std::chrono::microseconds;
using std::chrono::milliseconds;

// Much shorter than the ~33 ms between rangings, so no ranging is overwritten unread.
constexpr milliseconds POLL_PERIOD{2};

static FrontRangeReader reader;

static void print_range(const FrontRange &range)
{
    std::printf(
        "range,%lld,%u,%u\n",
        range.measured_at.count(),
        static_cast<unsigned>(range.distance_mm),
        static_cast<unsigned>(range.device_status)
    );
}

extern "C" void app_main()
{
    reader.start();
    std::printf("# front_range: kind,t_us,distance_mm,device_status\n");

    microseconds last_printed{-1};
    TickType_t last_wake = xTaskGetTickCount();
    while (true)
    {
        const std::optional<FrontRange> range = reader.latest_range();
        const bool is_new = range && range->measured_at != last_printed;
        if (is_new)
        {
            print_range(*range);
            last_printed = range->measured_at;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD.count()));
    }
}
