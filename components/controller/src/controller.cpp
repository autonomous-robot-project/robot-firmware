#include "controller.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/task.h>

#include "config.h"
#include "control/inverse_kinematics.h"

static std::chrono::microseconds now()
{
    return std::chrono::microseconds{esp_timer_get_time()};
}

Controller::Controller()
    : left_motor_(LEFT_MOTOR), right_motor_(RIGHT_MOTOR), left_encoder_(LEFT_ENCODER_PIN),
      right_encoder_(RIGHT_ENCODER_PIN)
{
}

void Controller::start()
{
    watchdog_.start();
    restarted_by_watchdog_ = TaskWatchdog::caused_last_restart();
    watchdog_test_trigger_.start();

    // One interrupt service for all GPIO interrupts, on the calling core (core 0).
    constexpr int default_flags = 0;
    const esp_err_t installed = gpio_install_isr_service(default_flags);
    ESP_ERROR_CHECK(installed);
    left_encoder_.start();
    right_encoder_.start();

    left_motor_.start();
    right_motor_.start();

    const BaseType_t created = xTaskCreatePinnedToCore(
        &Controller::task_entry,
        "control",
        TASK_STACK_BYTES,
        this,
        TASK_PRIORITY,
        nullptr,
        TASK_CORE
    );
    configASSERT(created == pdPASS);
}

void Controller::submit_command(const Command &command)
{
    commands_.write({.command = command, .arrival = now()});
}

std::optional<State> Controller::latest_state() const
{
    return states_.read();
}

std::optional<Instrumentation> Controller::latest_instrumentation() const
{
    return instrumentation_.read();
}

void Controller::task_entry(void *controller)
{
    static_cast<Controller *>(controller)->run();
}

void Controller::run()
{
    const TickType_t period = pdMS_TO_TICKS(T_C.count());
    watchdog_.watch_calling_task();
    TickType_t last_release = xTaskGetTickCount();
    // Waits for the first release, so that t_0 is a release instant and the nominal release
    // instants are t_k = t_0 + k * T_C (V-05).
    vTaskDelayUntil(&last_release, period);
    std::chrono::microseconds release = now();
    for (;;)
    {
        control_cycle(release);
        vTaskDelayUntil(&last_release, period);
        release += T_C;
    }
}

void Controller::control_cycle(std::chrono::microseconds release)
{
    watchdog_test_trigger_.hang_if_requested();

    const std::optional<StampedCommand> latest = commands_.read();
    // Decided once per cycle: every step below uses this copy, already expired if too old.
    const Command command = command_in_force(latest, release);

    const WheelPair measured = measure_wheel_speeds(release);
    const WheelPair references = wheel_references(command);
    drive_wheels(references, measured);

    const bool wheels_lost_tracking = tracking_lost(references, measured, release);
    publish_state(command, active_faults(release, wheels_lost_tracking));
    record_instrumentation(latest, release, measured);

    watchdog_.feed();
}

WheelPair Controller::measure_wheel_speeds(std::chrono::microseconds now)
{
    return {
        .left = left_speed_.update(left_encoder_.latest_edges(), now, applied_duty_.left),
        .right = right_speed_.update(right_encoder_.latest_edges(), now, applied_duty_.right),
    };
}

WheelPair Controller::wheel_references(const Command &command)
{
    const Twist target = target_twist(command);
    const Twist reference = acceleration_limiter_.limit(target);

    return inverse_kinematics(reference);
}

void Controller::drive_wheels(const WheelPair &references, const WheelPair &measured)
{
    const WheelPair duty = {
        .left = left_pi_.update(references.left, measured.left),
        .right = right_pi_.update(references.right, measured.right),
    };

    left_motor_.apply(duty.left);
    right_motor_.apply(duty.right);
    applied_duty_ = duty;
}

bool Controller::tracking_lost(
    const WheelPair &references, const WheelPair &measured, std::chrono::microseconds now
)
{
    // Both monitors run every cycle, so each one keeps its own timing.
    const bool left_lost = left_tracking_.update(references.left, measured.left, now);
    const bool right_lost = right_tracking_.update(references.right, measured.right, now);

    return left_lost || right_lost;
}

void Controller::publish_state(const Command &command, uint32_t faults)
{
    State state;
    state.faults = faults;
    state.echo = command;
    states_.write(state);
}

Twist Controller::target_twist(const Command &command)
{
    if (command.behavior != Behavior::TWIST)
    {
        return Twist{};
    }

    return {.linear = command.linear, .angular = command.angular};
}

uint32_t Controller::active_faults(std::chrono::microseconds now, bool tracking_lost) const
{
    uint32_t faults = 0;

    if (tracking_lost)
    {
        faults |= State::FAULT_COGNIZANT;
    }

    // `now` counts from the boot. The code stays active long enough for the sequencer to see it,
    // even after a slow reconnection of the session (ADR 029).
    const bool watchdog_restart_recent = restarted_by_watchdog_ && now < WATCHDOG_FAULT_DURATION;
    if (watchdog_restart_recent)
    {
        faults |= State::FAULT_WATCHDOG_RESTART;
    }

    return faults;
}

Command Controller::command_in_force(
    const std::optional<StampedCommand> &latest, std::chrono::microseconds now
)
{
    // An empty channel counts as an expired command (ADR 017).
    if (!latest || latest->expired(now))
    {
        return Command::stop();
    }

    return latest->command;
}

bool Controller::StampedCommand::expired(std::chrono::microseconds now) const
{
    return now - arrival > T_CMD;
}

void Controller::record_instrumentation(
    const std::optional<StampedCommand> &latest,
    std::chrono::microseconds cycle,
    const WheelPair &wheel_speeds
)
{
    if (latest && latest->expired(cycle))
    {
        expiry_meter_.record(latest->arrival, cycle);
    }

    Instrumentation instrumentation = expiry_meter_.measurements();
    instrumentation.left_wheel_speed = wheel_speeds.left;
    instrumentation.right_wheel_speed = wheel_speeds.right;
    instrumentation_.write(instrumentation);
}
