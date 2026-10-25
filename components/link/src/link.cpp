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
    TickType_t last_release = xTaskGetTickCount();
    TickType_t last_ping = last_release;
    for (;;)
    {
        session.receive_commands();
        session.publish_state();

        if (xTaskGetTickCount() - last_ping >= to_ticks(AGENT_PING_PERIOD))
        {
            last_ping = xTaskGetTickCount();
            if (!agent.is_alive())
            {
                return;
            }
        }
        vTaskDelayUntil(&last_release, to_ticks(STATE_PERIOD));
    }
}
