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
    TickType_t last_ping = xTaskGetTickCount();
    for (;;)
    {
        // Each command reaches the controller as soon as it arrives, so the controller stamps
        // its reception time (ADR 017, RNF-02); the state goes out every STATE_PERIOD.
        session.receive_commands_until(next_publication);
        session.publish_state();
        next_publication = next_deadline(next_publication, state_period);

        if (xTaskGetTickCount() - last_ping >= to_ticks(AGENT_PING_PERIOD))
        {
            last_ping = xTaskGetTickCount();
            if (!agent.is_alive())
            {
                return;
            }
        }
    }
}
