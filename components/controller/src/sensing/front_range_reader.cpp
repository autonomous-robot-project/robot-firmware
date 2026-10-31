// Criticality: T2, product (robot-docs codigo.md).
#include "sensing/front_range_reader.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

static const char *TAG = "front_range";

FrontRangeReader::FrontRangeReader()
    : sensor_({.sda = FRONT_RANGE_SDA_PIN, .scl = FRONT_RANGE_SCL_PIN})
{
}

void FrontRangeReader::start()
{
    sensor_.start();

    const BaseType_t created = xTaskCreatePinnedToCore(
        &FrontRangeReader::task_entry,
        "front_range",
        SENSOR_TASK_STACK_BYTES,
        this,
        SENSOR_TASK_PRIORITY,
        nullptr,
        SENSOR_TASK_CORE
    );
    configASSERT(created == pdPASS);
}

void FrontRangeReader::task_entry(void *reader)
{
    static_cast<FrontRangeReader *>(reader)->run();
}

void FrontRangeReader::run()
{
    while (true)
    {
        if (bring_up())
        {
            read_until_failure();
        }
        recover();
    }
}

bool FrontRangeReader::bring_up()
{
    const esp_err_t initialized = sensor_.initialize();
    if (initialized != ESP_OK)
    {
        ESP_LOGW(TAG, "initialization failed: %s", esp_err_to_name(initialized));
        return false;
    }
    const esp_err_t started = sensor_.start_continuous();
    if (started != ESP_OK)
    {
        ESP_LOGW(TAG, "continuous mode failed: %s", esp_err_to_name(started));
        return false;
    }
    ESP_LOGI(TAG, "ranging");
    return true;
}

void FrontRangeReader::read_until_failure()
{
    int consecutive_failures = 0;
    while (consecutive_failures < N_I2C)
    {
        VL53L0X::Measurement measurement{};
        const esp_err_t read = sensor_.read_measurement(measurement);
        if (read != ESP_OK)
        {
            ++consecutive_failures;
            ESP_LOGW(TAG, "reading failed: %s", esp_err_to_name(read));
            continue;
        }
        consecutive_failures = 0;
        ranges_.write({
            .distance_mm = measurement.distance_mm,
            .device_status = measurement.device_status,
            .signal_rate_q9_7 = measurement.signal_rate_q9_7,
            .effective_spads_q8_8 = measurement.effective_spads_q8_8,
            .measured_at = std::chrono::microseconds{esp_timer_get_time()},
        });
    }
}

void FrontRangeReader::recover()
{
    vTaskDelay(pdMS_TO_TICKS(T_RETRY.count()));
    const esp_err_t reset = sensor_.reset_bus();
    if (reset != ESP_OK)
    {
        ESP_LOGW(TAG, "bus reset failed: %s", esp_err_to_name(reset));
    }
}
