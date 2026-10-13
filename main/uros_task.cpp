#include "uros_task.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "robot_config.h"

static const char *TAG = "uros";

static rcl_publisher_t s_pub;
static std_msgs__msg__Int32 s_msg;
static int32_t s_count;

static void timer_cb(rcl_timer_t *timer, int64_t)
{
    if (timer == nullptr) {
        return;
    }
    // Smoke test: a dezena de milhar leva o núcleo de execução (0 esperado), o resto é o contador.
    s_msg.data = xPortGetCoreID() * SMOKE_CORE_FACTOR + s_count++;
    if (rcl_publish(&s_pub, &s_msg, nullptr) == RCL_RET_OK) {
        ESP_LOGI(TAG, "core %d: publicou %ld", xPortGetCoreID(), static_cast<long>(s_msg.data));
    }
}

// Uma sessão completa: cria entidades, gira o executor, destrói. Retorna ao perder o agente.
static void run_session()
{
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t exec;

    if (rclc_support_init(&support, 0, nullptr, &alloc) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init falhou");
        return;
    }
    if (rclc_node_init_default(&node, "robot_controller", "", &support) != RCL_RET_OK ||
        rclc_publisher_init_default(&s_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                    "smoke_counter") != RCL_RET_OK ||
        rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(SMOKE_PUB_PERIOD_MS), timer_cb, true) != RCL_RET_OK ||
        rclc_executor_init(&exec, &support.context, 1, &alloc) != RCL_RET_OK ||
        rclc_executor_add_timer(&exec, &timer) != RCL_RET_OK) {
        ESP_LOGE(TAG, "falha ao criar entidades");
        rclc_support_fini(&support);
        return;
    }

    ESP_LOGI(TAG, "sessão ativa no core %d", xPortGetCoreID());
    s_count = 0;
    // Ping periódico: spin_some sozinho não percebe a queda do agente.
    while (rmw_uros_ping_agent(UROS_AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
        rclc_executor_spin_some(&exec, RCL_MS_TO_NS(UROS_SPIN_TIMEOUT_MS));
    }
    ESP_LOGW(TAG, "agente perdido");

    rclc_executor_fini(&exec);
    rcl_timer_fini(&timer);
    rcl_publisher_fini(&s_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

static void uros_task(void *)
{
    for (;;) {
        ESP_LOGI(TAG, "aguardando agente");
        while (rmw_uros_ping_agent(UROS_AGENT_PING_TIMEOUT_MS, UROS_AGENT_PING_TRIES) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(UROS_RETRY_DELAY_MS));
        }
        run_session();
    }
}

void uros_task_start()
{
    xTaskCreatePinnedToCore(uros_task, "uros", UROS_TASK_STACK_BYTES, nullptr, UROS_TASK_PRIO, nullptr, CORE_UROS);
}
