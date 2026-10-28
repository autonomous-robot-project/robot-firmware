#include "link.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agent.h"
#include "config.h"
#include "session.h"

static const char *TAG = "link";

static TickType_t to_ticks(std::chrono::milliseconds duration)
{
    return pdMS_TO_TICKS(duration.count());
}

// The deadline after `deadline`, one period later; if the task fell behind (a ping takes up to
// its timeout), the next one is a period from now, so late publications do not come in a burst.
static TickType_t next_deadline(TickType_t deadline, TickType_t period)
{
    const TickType_t now = xTaskGetTickCount();
    const bool behind = static_cast<int32_t>(now - deadline) >= static_cast<int32_t>(period);
    return (behind ? now : deadline) + period;
}

// True once `period` has passed since `last`, which then moves to now.
static bool is_due(TickType_t &last, std::chrono::milliseconds period)
{
    if (xTaskGetTickCount() - last < to_ticks(period))
    {
        return false;
    }
    last = xTaskGetTickCount();
    return true;
}

Link::Link(Controller &controller) : controller_(controller), transport_(UART_CONFIG)
{
}

void Link::start()
{
    transport_.register_with_micro_ros();
    const BaseType_t created = xTaskCreatePinnedToCore(
        &Link::task_entry, "link", TASK_STACK_BYTES, this, TASK_PRIORITY, nullptr, TASK_CORE
    );
    configASSERT(created == pdPASS);
}

void Link::task_entry(void *link)
{
    static_cast<Link *>(link)->run();
}

void Link::run()
{
    const Agent agent;
    for (;;)
    {
        ESP_LOGI(TAG, "waiting for agent");
        agent.wait_until_alive();
        Session session{controller_};
        if (!session.ok())
        {
            vTaskDelay(to_ticks(AGENT_RETRY_PERIOD));
            continue;
        }
        ESP_LOGI(TAG, "session up");
        serve(session, agent);
        ESP_LOGW(TAG, "agent lost");
    }
}

void Link::serve(Session &session, const Agent &agent)
{
    const TickType_t state_period = to_ticks(STATE_PERIOD);
    TickType_t next_publication = xTaskGetTickCount() + state_period;
    TickType_t last_instrumentation = xTaskGetTickCount();
    TickType_t last_odometry = xTaskGetTickCount();
    TickType_t last_clock_sync = xTaskGetTickCount();
    TickType_t last_ping = xTaskGetTickCount();
    // The odometry waits for the first synchronization (ADR 027).
    session.synchronize_clock();
    for (;;)
    {
        // Each command reaches the controller as soon as it arrives, so the controller stamps
        // its reception time (ADR 017, RNF-02); the state goes out every STATE_PERIOD.
        session.receive_commands_until(next_publication);
        session.publish_state();
        next_publication = next_deadline(next_publication, state_period);

        if (is_due(last_instrumentation, INSTRUMENTATION_PERIOD))
        {
            session.publish_instrumentation();
        }
        if (is_due(last_odometry, ODOMETRY_PERIOD))
        {
            session.publish_odometry();
        }
        if (is_due(last_clock_sync, CLOCK_SYNC_PERIOD))
        {
            session.synchronize_clock();
        }
        if (is_due(last_ping, AGENT_PING_PERIOD) && !agent.is_alive())
        {
            return;
        }
    }
}
