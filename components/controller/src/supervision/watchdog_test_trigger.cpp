#include "supervision/watchdog_test_trigger.h"

#include <driver/gpio.h>
#include <esp_err.h>

#include "config.h"

void WatchdogTestTrigger::start()
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ull << WATCHDOG_TEST_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const esp_err_t configured = gpio_config(&config);
    ESP_ERROR_CHECK(configured);
}

void WatchdogTestTrigger::hang_if_requested() const
{
    const bool requested = gpio_get_level(WATCHDOG_TEST_PIN) == 0;
    if (!requested)
    {
        return;
    }

    // The injected defect: a cycle that never ends. The read of a volatile is what keeps the
    // loop, because an endless loop without side effects is undefined behaviour in C++.
    volatile bool hung = true;
    while (hung)
    {
    }
}
