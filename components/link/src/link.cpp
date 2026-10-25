#include "link.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"
#include "transport.h"

namespace {

const char *TAG = "link";

bool agent_alive()
{
    return rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK;
}

void wait_for_agent()
{
    while (!agent_alive()) {
        vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_PERIOD_MS));
    }
}

// One session: creates the entities, keeps them while the agent answers, then destroys them.
// Losing the session is not a safety event: the controller stops by command expiry (RNF-02).
void run_session()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
        ESP_LOGE(TAG, "support init failed");
        return;
    }
    rcl_node_t node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
        ESP_LOGE(TAG, "node init failed");
        rclc_support_fini(&support);
        return;
    }
    ESP_LOGI(TAG, "session up");

    TickType_t last_check = xTaskGetTickCount();
    do {
        vTaskDelayUntil(&last_check, pdMS_TO_TICKS(AGENT_PING_PERIOD_MS));
    } while (agent_alive());
    ESP_LOGW(TAG, "agent lost");

    // The agent is gone: destroy without waiting for acknowledgements that will never come.
    // Errors are expected here, since the agent cannot confirm the destruction.
    rmw_uros_set_context_entity_destroy_session_timeout(rcl_context_get_rmw_context(&support.context), 0);
    const rcl_ret_t node_ret = rcl_node_fini(&node);
    const rcl_ret_t support_ret = rclc_support_fini(&support);
    ESP_LOGD(TAG, "session destroyed (node %d, support %d)", static_cast<int>(node_ret),
             static_cast<int>(support_ret));
}

void link_task(void *)
{
    for (;;) {
        ESP_LOGI(TAG, "waiting for agent");
        wait_for_agent();
        run_session();
    }
}

}  // namespace

void link_start()
{
    transport_register();
    const BaseType_t created =
        xTaskCreatePinnedToCore(link_task, "link", TASK_STACK_BYTES, nullptr, TASK_PRIORITY, nullptr, TASK_CORE);
    configASSERT(created == pdPASS);
}
