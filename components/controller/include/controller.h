#pragma once

#include <chrono>
#include <optional>

#include "interface/command.h"
#include "instrumentation/expiry_meter.h"
#include "instrumentation/instrumentation.h"
#include "channels/latest_value_channel.h"
#include "actuation/motor_driver.h"
#include "interface/state.h"
#include "sensing/wheel_encoder.h"
#include "sensing/wheel_speed.h"
#include "supervision/watchdog_test_trigger.h"
#include "supervision/task_watchdog.h"

// Controller layer (ADR 018): the real-time control task on core 1 and its latest-value channels.
// It does not depend on micro-ROS; the link feeds the command channel and reads the state channel.
class Controller
{
    public:
        Controller();
        Controller(const Controller &) = delete;
        Controller &operator=(const Controller &) = delete;

        // Starts the watchdog, the watchdog test trigger, the encoders and the motor drivers, with
        // the wheels free, and then the control task. Called once, at boot, on core 0.
        void start();

        // Producer side of the command channel, called by the link. Stamps the arrival time,
        // from which the control task expires the command (ADR 017, RNF-02).
        void submit_command(const Command &command);

        // Consumer side of the state channel, called by the link. Empty before the first cycle.
        [[nodiscard]] std::optional<State> latest_state() const;

        // Consumer side of the instrumentation channel (ADR 028). Empty before the first cycle.
        [[nodiscard]] std::optional<Instrumentation> latest_instrumentation() const;

    private:
        // Measured speed of each wheel in one cycle, in m/s (V-02).
        struct WheelSpeeds
        {
            public:
                float left = 0.0f;
                float right = 0.0f;
        };

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

        // One cycle at its nominal release instant t_k (V-05). The expiry is decided and measured
        // at t_k, so it happens within T_CMD + T_C of the reception (RNF-02).
        void control_cycle(std::chrono::microseconds release);

        // The latest command, or TWIST(0, 0) if none arrived or it is older than T_CMD.
        [[nodiscard]] static Command command_in_force(
            const std::optional<StampedCommand> &latest, std::chrono::microseconds now
        );

        // Faults active at `now`, one bit per code (State::FAULT_*).
        [[nodiscard]] uint32_t active_faults(std::chrono::microseconds now) const;

        // Measurements of the cycle for the verifications (ADR 028); never read by the control.
        void measure(
            const std::optional<StampedCommand> &latest,
            std::chrono::microseconds cycle,
            const WheelSpeeds &wheel_speeds
        );

        LatestValueChannel<StampedCommand> commands_;
        LatestValueChannel<State> states_;
        LatestValueChannel<Instrumentation> instrumentation_;
        ExpiryMeter expiry_meter_;
        MotorDriver left_motor_;
        MotorDriver right_motor_;
        WheelEncoder left_encoder_;
        WheelEncoder right_encoder_;
        WheelSpeed left_speed_;
        WheelSpeed right_speed_;
        TaskWatchdog watchdog_;
        WatchdogTestTrigger watchdog_test_trigger_;
        bool restarted_by_watchdog_ = false;
};
