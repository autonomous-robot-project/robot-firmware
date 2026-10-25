#include "link.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agent.h"
#include "config.h"
#include "session.h"

static const char *TAG = "link";

Link::Link() : transport_(UART_CONFIG)
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
        const Session session;
        if (!session.ok())
        {
            vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_PERIOD.count()));
            continue;
        }
        ESP_LOGI(TAG, "session up");
        agent.wait_until_lost();
        ESP_LOGW(TAG, "agent lost");
    }
}
