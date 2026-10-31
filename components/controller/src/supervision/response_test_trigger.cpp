#include "supervision/response_test_trigger.h"

#include <driver/gpio.h>
#include <esp_err.h>

#include "config.h"

void ResponseTestTrigger::start()
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ull << RESPONSE_TEST_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const esp_err_t configured = gpio_config(&config);
    ESP_ERROR_CHECK(configured);
}

bool ResponseTestTrigger::requested() const
{
    return gpio_get_level(RESPONSE_TEST_PIN) == 0;
}
