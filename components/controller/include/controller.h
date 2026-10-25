#pragma once

#include <chrono>
#include <optional>

#include "command.h"
#include "latest_value_channel.h"
#include "state.h"

// Controller layer (ADR 018): the real-time control task on core 1 and its latest-value channels.
// It does not depend on micro-ROS; the link feeds the command channel and reads the state channel.
class Controller
{
    public:
        Controller() = default;
        Controller(const Controller &) = delete;
        Controller &operator=(const Controller &) = delete;

        // Starts the control task. Called once, at boot.
        void start();

        // Producer side of the command channel, called by the link. Stamps the arrival time,
        // from which the control task expires the command (ADR 017, RNF-02).
        void submit_command(const Command &command);

        // Consumer side of the state channel, called by the link. Empty before the first cycle.
        [[nodiscard]] std::optional<State> latest_state() const;

    private:
        struct StampedCommand
        {
            public:
                Command command;
                std::chrono::microseconds arrival;
        };

        static void task_entry(void *controller);

        // Task body: one control cycle every T_C. A FreeRTOS task must never return.
        [[noreturn]] void run();

        void control_cycle();

        // The received command, or TWIST(0, 0) if none arrived within T_CMD.
        [[nodiscard]] Command command_in_force(std::chrono::microseconds now) const;

        LatestValueChannel<StampedCommand> commands_;
        LatestValueChannel<State> states_;
};
