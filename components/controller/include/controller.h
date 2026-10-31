#pragma once

#include <chrono>
#include <optional>

#include "interface/command.h"
#include "instrumentation/expiry_meter.h"
#include "instrumentation/instrumentation.h"
#include "channels/latest_value_channel.h"
#include "control/acceleration_limiter.h"
#include "control/pose.h"
#include "control/twist.h"
#include "control/wheel_pair.h"
#include "control/wheel_pi.h"
#include "actuation/motor_driver.h"
#include "interface/odometry.h"
#include "interface/state.h"
#include "sensing/front_range_reader.h"
#include "sensing/wheel_encoder.h"
#include "sensing/wheel_odometry.h"
#include "sensing/wheel_speed.h"
#include "supervision/watchdog_test_trigger.h"
#include "supervision/wheel_tracking_monitor.h"
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
        // the wheels free, then the sensor task and the control task. Called once, at boot, on
        // core 0.
        void start();

        // Producer side of the command channel, called by the link. Stamps the arrival time,
        // from which the control task expires the command (ADR 017, RNF-02).
        void submit_command(const Command &command);

        // Consumer side of the state channel, called by the link. Empty before the first cycle.
        [[nodiscard]] std::optional<State> latest_state() const;

        // Consumer side of the instrumentation channel (ADR 028). Empty before the first cycle.
        [[nodiscard]] std::optional<Instrumentation> latest_instrumentation() const;

        // Consumer side of the odometry channel (ADR 048). Empty before the first cycle.
        [[nodiscard]] std::optional<Odometry> latest_odometry() const;

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

        // One cycle at its nominal release instant t_k (V-05). The expiry is decided and measured
        // at t_k, so it happens within T_CMD + T_C of the reception (RNF-02).
        void control_cycle(std::chrono::microseconds release);

        // The latest command, or TWIST(0, 0) if none arrived or it is older than T_CMD.
        [[nodiscard]] static Command command_in_force(
            const std::optional<StampedCommand> &latest, std::chrono::microseconds now
        );

        // The twist that the active behaviour asks for (ADR 024). Computed every cycle from the
        // command in force and the pose, and kept nowhere: it ends with the behaviour (RNF-02).
        [[nodiscard]] static Twist target_twist(const Command &command, const Pose &pose);

        // Faults active at `now`, one bit per code (State::FAULT_*).
        [[nodiscard]] uint32_t active_faults(std::chrono::microseconds now, bool tracking_lost)
            const;

        // The latest edges of both encoders, read once per cycle.
        [[nodiscard]] WheelEdges read_wheel_edges() const;

        // Speed of each wheel from its edges, with the direction of the previous duty (ADR 019).
        [[nodiscard]] WheelPair measure_wheel_speeds(
            const WheelEdges &edges, std::chrono::microseconds now
        );

        // Integrates the new edges of each wheel into the pose (ADR 048) and returns it.
        [[nodiscard]] Pose update_odometry(const WheelEdges &edges);

        // Speed of each wheel that the active behaviour asks for, after the acceleration limits
        // (ADR 023).
        [[nodiscard]] WheelPair wheel_references(const Command &command, const Pose &pose);

        // PI of each wheel and the PWM of its motor.
        void drive_wheels(const WheelPair &references, const WheelPair &measured);

        // True if either wheel has lost the tracking of its reference (RF-02).
        [[nodiscard]] bool tracking_lost(
            const WheelPair &references, const WheelPair &measured, std::chrono::microseconds now
        );

        void publish_state(const Command &command, uint32_t faults);

        // Writes the pose and the measured twist, stamped with the release of the cycle (ADR 027).
        // The edges are read tens of us after t_k, far below the ~51 ms between edges at 0.2 m/s.
        void publish_odometry(
            const Pose &pose, const WheelPair &wheel_speeds, std::chrono::microseconds cycle
        );

        // Measures the expiry of the latest command for V-06; never read by the control.
        void record_expiry(
            const std::optional<StampedCommand> &latest, std::chrono::microseconds cycle
        );

        // Writes the measurements of the meters for the verifications (ADR 028).
        void publish_instrumentation(const WheelPair &wheel_speeds);

        LatestValueChannel<StampedCommand> commands_;
        LatestValueChannel<State> states_;
        LatestValueChannel<Instrumentation> instrumentation_;
        LatestValueChannel<Odometry> odometry_;
        ExpiryMeter expiry_meter_;
        DeadlineMeter deadline_meter_;
        MotorDriver left_motor_;
        MotorDriver right_motor_;
        // Producer of the front range channel; read by the reflex (ADR 021).
        FrontRangeReader front_range_reader_;
        WheelEncoder left_encoder_;
        WheelEncoder right_encoder_;
        WheelSpeed left_speed_;
        WheelSpeed right_speed_;
        WheelOdometry wheel_odometry_;
        AccelerationLimiter acceleration_limiter_;
        WheelPI left_pi_;
        WheelPI right_pi_;
        WheelTrackingMonitor left_tracking_;
        WheelTrackingMonitor right_tracking_;
        // Duty of the previous cycle: the direction of a stopped wheel (ADR 019).
        WheelPair applied_duty_;
        TaskWatchdog watchdog_;
        WatchdogTestTrigger watchdog_test_trigger_;
        bool restarted_by_watchdog_ = false;
};
