#include "controller.h"

#include <algorithm>

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/task.h>

#include "config.h"
#include "control/forward_kinematics.h"
#include "control/go_to.h"
#include "control/inverse_kinematics.h"
#include "control/turn_to.h"

static std::chrono::microseconds now()
{
    return std::chrono::microseconds{esp_timer_get_time()};
}

// Edges since boot; zero before the first slot, as at boot.
static uint32_t edge_count(const std::optional<EncoderEdges> &edges)
{
    return edges ? edges->count : 0;
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

    front_range_reader_.start();

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

std::optional<Odometry> Controller::latest_odometry() const
{
    return odometry_.read();
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
    const std::chrono::microseconds start = now();
    watchdog_test_trigger_.hang_if_requested();

    const std::optional<StampedCommand> latest = commands_.read();
    // Decided once per cycle: every step below uses this copy, already expired if too old.
    const Command command = command_in_force(latest, release);
    const std::optional<FrontRange> range = front_range_reader_.latest_range();
    const FrontCondition front = front_range_classifier_.classify(range, release);

    const WheelEdges edges = read_wheel_edges();
    const WheelPair measured = measure_wheel_speeds(edges, release);
    const Pose pose = update_odometry(edges);
    const Twist target = restrict_by_reflexes(target_twist(command, pose), front);
    const WheelPair references = wheel_references(target);
    drive_wheels(references, measured);

    const bool wheels_lost_tracking = tracking_lost(references, measured, release);
    publish_state(command, release, wheels_lost_tracking, front);
    publish_odometry(pose, measured, release);

    // Instrumentation logic
    record_expiry(latest, release);
    publish_instrumentation(measured);

    watchdog_.feed();

    // Last step, so C and R cover the whole cycle, the instrumentation included; this cycle
    // reaches the instrumentation in the next one, which is enough for values accumulated since
    // boot (V-05).
    deadline_meter_.record(release, start, now());
}

WheelEdges Controller::read_wheel_edges() const
{
    return {.left = left_encoder_.latest_edges(), .right = right_encoder_.latest_edges()};
}

WheelPair Controller::measure_wheel_speeds(const WheelEdges &edges, std::chrono::microseconds now)
{
    return {
        .left = left_speed_.update(edges.left, now, applied_duty_.left),
        .right = right_speed_.update(edges.right, now, applied_duty_.right),
    };
}

Pose Controller::update_odometry(const WheelEdges &edges)
{
    // The directions locked by measure_wheel_speeds in this cycle (ADR 019).
    const WheelPair directions = {
        .left = left_speed_.direction(),
        .right = right_speed_.direction(),
    };
    const EdgeCounts counts = {
        .left = edge_count(edges.left),
        .right = edge_count(edges.right),
    };
    wheel_odometry_.update(counts, directions);

    return wheel_odometry_.pose();
}

WheelPair Controller::wheel_references(const Twist &target)
{
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

void Controller::publish_state(
    const Command &command, std::chrono::microseconds now, bool tracking_lost, FrontCondition front
)
{
    State state;
    state.faults = active_faults(now, tracking_lost, front);
    state.reflexes = active_reflexes(front);
    state.echo = command;
    states_.write(state);
}

void Controller::publish_odometry(
    const Pose &pose, const WheelPair &wheel_speeds, std::chrono::microseconds cycle
)
{
    odometry_.write({
        .pose = pose,
        .twist = forward_kinematics(wheel_speeds),
        .instant = cycle,
    });
}

Twist Controller::target_twist(const Command &command, const Pose &pose)
{
    switch (command.behavior)
    {
        case Behavior::TWIST:
            return {.linear = command.linear, .angular = command.angular};
        case Behavior::TURN_TO:
            return turn_to(command.theta, pose);
        case Behavior::GO_TO:
            return go_to({.x = command.x, .y = command.y}, pose);
    }

    return Twist{};  // unreachable: every behaviour is handled above
}

uint32_t Controller::active_faults(
    std::chrono::microseconds now, bool tracking_lost, FrontCondition front
) const
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

    if (front == FrontCondition::FAILURE)
    {
        faults |= State::FAULT_FRONT_SENSOR;
    }

    return faults;
}

uint32_t Controller::active_reflexes(FrontCondition front)
{
    return front == FrontCondition::OBSTACLE ? State::REFLEX_FRONT_OBSTACLE : 0;
}

Twist Controller::restrict_by_reflexes(const Twist &target, FrontCondition front)
{
    // A failed sensor restricts as an obstacle does. The sensor covers only the path ahead, so
    // turning and reversing stay allowed (ADR 021).
    if (front == FrontCondition::CLEAR)
    {
        return target;
    }

    return {.linear = std::min(target.linear, 0.0f), .angular = target.angular};
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

void Controller::record_expiry(
    const std::optional<StampedCommand> &latest, std::chrono::microseconds cycle
)
{
    const bool latest_expired = latest && latest->expired(cycle);
    if (latest_expired)
    {
        expiry_meter_.record(latest->arrival, cycle);
    }
}

void Controller::publish_instrumentation(const WheelPair &wheel_speeds)
{
    instrumentation_.write({
        .expiry = expiry_meter_.measurements(),
        .deadlines = deadline_meter_.measurements(),
        .wheel_speeds = wheel_speeds,
    });
}
