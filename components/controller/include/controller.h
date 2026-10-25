#pragma once

#include <chrono>
#include <optional>

#include "command.h"
#include "expiry_meter.h"
#include "instrumentation.h"
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

        // Consumer side of the instrumentation channel (ADR 028). Empty before the first cycle.
        [[nodiscard]] std::optional<Instrumentation> latest_instrumentation() const;

    private:
        struct StampedCommand
        {
            public:
                Command command;
                std::chrono::microseconds arrival;

                // True if older than T_CMD (RNF-02).
                [[nodiscard]] bool expired(std::chrono::microseconds now) const;
        };

        static void task_entry(void *controller);

        // Task body: one control cycle every T_C. A FreeRTOS task must never return.
        [[noreturn]] void run();

        void control_cycle();

        // The latest command, or TWIST(0, 0) if none arrived or it is older than T_CMD.
        [[nodiscard]] static Command command_in_force(
            const std::optional<StampedCommand> &latest, std::chrono::microseconds now
        );

        // Measurements of the cycle for the verifications (ADR 028); never read by the control.
        void measure(const std::optional<StampedCommand> &latest, std::chrono::microseconds cycle);

        LatestValueChannel<StampedCommand> commands_;
        LatestValueChannel<State> states_;
        LatestValueChannel<Instrumentation> instrumentation_;
        ExpiryMeter expiry_meter_;
};
