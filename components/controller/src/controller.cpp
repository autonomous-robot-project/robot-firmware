#include "controller.h"

#include <esp_timer.h>
#include <freertos/task.h>

#include "config.h"

static std::chrono::microseconds now()
{
    return std::chrono::microseconds{esp_timer_get_time()};
}

Controller::Controller() : left_motor_(LEFT_MOTOR), right_motor_(RIGHT_MOTOR)
{
}

void Controller::start()
{
    watchdog_.start();
    restarted_by_watchdog_ = TaskWatchdog::caused_last_restart();
    watchdog_test_trigger_.start();
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

    // Until the PI exists, the only reference is the stop: both wheels brake.
    constexpr float brake = 0.0f;
    left_motor_.apply(brake);
    right_motor_.apply(brake);

    State state;
    state.faults = active_faults(release);
    state.echo = command;
    states_.write(state);

    measure(latest, release);
    watchdog_.feed();
}

uint32_t Controller::active_faults(std::chrono::microseconds now) const
{
    uint32_t faults = 0;
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

void Controller::measure(
    const std::optional<StampedCommand> &latest, std::chrono::microseconds cycle
)
{
    if (latest && latest->expired(cycle))
    {
        expiry_meter_.record(latest->arrival, cycle);
    }
    instrumentation_.write(expiry_meter_.measurements());
}
