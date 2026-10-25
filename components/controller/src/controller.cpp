#include "controller.h"

#include <esp_timer.h>
#include <freertos/task.h>

#include "config.h"

static std::chrono::microseconds now()
{
    return std::chrono::microseconds{esp_timer_get_time()};
}

void Controller::start()
{
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

void Controller::task_entry(void *controller)
{
    static_cast<Controller *>(controller)->run();
}

void Controller::run()
{
    TickType_t last_release = xTaskGetTickCount();
    for (;;)
    {
        control_cycle();
        vTaskDelayUntil(&last_release, pdMS_TO_TICKS(T_C.count()));
    }
}

void Controller::control_cycle()
{
    // Read once per cycle: every step below uses this copy, already expired if too old.
    const Command command = command_in_force(now());

    State state;
    state.echo = command;
    states_.write(state);
}

Command Controller::command_in_force(std::chrono::microseconds now) const
{
    const std::optional<StampedCommand> latest = commands_.read();
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
