#include "agent.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"

static TickType_t to_ticks(std::chrono::milliseconds duration)
{
    return pdMS_TO_TICKS(duration.count());
}

bool Agent::is_alive() const
{
    constexpr int attempts = 1;
    const auto timeout_ms = static_cast<int>(AGENT_PING_TIMEOUT.count());
    return rmw_uros_ping_agent(timeout_ms, attempts) == RMW_RET_OK;
}

void Agent::wait_until_alive() const
{
    while (!is_alive())
    {
        vTaskDelay(to_ticks(AGENT_RETRY_PERIOD));
    }
}

void Agent::wait_until_lost() const
{
    TickType_t last_check = xTaskGetTickCount();
    do
    {
        vTaskDelayUntil(&last_check, to_ticks(AGENT_PING_PERIOD));
    } while (is_alive());
}
