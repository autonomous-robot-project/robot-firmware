#include "supervision/task_watchdog.h"

#include <esp_err.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "config.h"

void TaskWatchdog::start()
{
    constexpr uint32_t no_idle_task = 0;
    const esp_task_wdt_config_t config = {
        .timeout_ms = static_cast<uint32_t>(WATCHDOG_TIMEOUT.count()),
        .idle_core_mask = no_idle_task,
        .trigger_panic = true,
    };
    const esp_err_t initialized = esp_task_wdt_init(&config);
    ESP_ERROR_CHECK(initialized);
}

void TaskWatchdog::watch_calling_task()
{
    constexpr TaskHandle_t calling_task = nullptr;
    const esp_err_t added = esp_task_wdt_add(calling_task);
    ESP_ERROR_CHECK(added);
}

void TaskWatchdog::feed()
{
    const esp_err_t fed = esp_task_wdt_reset();
    ESP_ERROR_CHECK(fed);
}

bool TaskWatchdog::caused_last_restart()
{
    return esp_reset_reason() == ESP_RST_TASK_WDT;
}
