#pragma once

#include <optional>

#include "channels/latest_value_channel.h"
#include "sensing/front_range.h"
#include "sensing/vl53l0x.h"

// Sensor task of the controller (ADR 020): reads the VL53L0X at the pace of the sensor, on core 0
// above the micro-ROS task, and publishes each ranging to the front range channel (ADR 017).
// After N_I2C consecutive failures it blocks for T_RETRY, resets the bus and initializes the
// sensor again (robot-docs nota 0015). It is not real time: while it fails, the latest ranging
// ages, and the control task detects the failure by its age (ADR 021).
// Criticality: T2, product (robot-docs codigo.md).
class FrontRangeReader
{
    public:
        FrontRangeReader();
        FrontRangeReader(const FrontRangeReader &) = delete;
        FrontRangeReader &operator=(const FrontRangeReader &) = delete;

        // Creates the I2C bus and starts the sensor task. Called once, at boot, from core 0, so
        // that the interrupt of the bus stays off the core of the control task.
        void start();

        // Consumer side, called by the control task. Empty before the first ranging.
        [[nodiscard]] std::optional<FrontRange> latest_range() const { return ranges_.read(); }

    private:
        static void task_entry(void *reader);

        // Task body: brings the sensor up and reads it until it fails, forever.
        [[noreturn]] void run();

        // Initializes the sensor and starts its continuous mode.
        [[nodiscard]] bool bring_up();

        // Publishes each ranging until N_I2C consecutive readings fail.
        void read_until_failure();

        // Waits T_RETRY and releases the bus before the next bring-up.
        void recover();

        VL53L0X sensor_;
        LatestValueChannel<FrontRange> ranges_;
};
