// Bench tool (robot-docs nota 0010): the analog outputs (AO) of the two encoders on each duty step,
// forward then backward, against the slots counted by the firmware. Wheels suspended, AO of the
// left encoder on GPIO 6 and of the right one on GPIO 7. For each step: applies the duty and
// samples both AO on core 1 into RAM from the step instant while core 0 reports the slots, brakes,
// then dumps the step on the USB console:
//   duty,<t_us>,<duty>
//   edge,<t_us>,<L|R>,<count>
//   ao,<t_us>,<left raw>,<right raw>   12-bit ADC counts, 12 dB attenuation
//   done
// The steps above DUTY_MAX are clamped by MotorDriver: the run of 2026-10-27 was built with
// DUTY_MAX = 0.55 in control_config.h, only for that build, to reach 6 V and 6.5 V on the motor.
// Recorded on the RPi with the record command of tools/motor_characterization/characterize.py and
// analyzed on the notebook with tools/encoder_ao/analyze_encoder_ao.py.

#include <chrono>
#include <cstdio>
#include <cstdlib>

#include <driver/gpio.h>
#include <driver/usb_serial_jtag.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_vfs_dev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "actuation/motor_driver.h"
#include "edge_reporter.h"
#include "pins.h"
#include "sensing/wheel_encoder.h"

constexpr float STEP_DUTIES[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.48f, 0.504f, 0.546f};
constexpr float BRAKE = 0.0f;
constexpr int BRAKE_MS = 2000;
constexpr int POLL_MS = 2;
constexpr adc_channel_t LEFT_AO = ADC_CHANNEL_5;   // GPIO 6
constexpr adc_channel_t RIGHT_AO = ADC_CHANNEL_6;  // GPIO 7
constexpr int SAMPLE_PERIOD_US = 250;              // 4 kHz per channel
constexpr int SAMPLES = 12000;                     // 3 s, under the 5 s of the task watchdog

struct AoSample
{
        uint32_t time_us;
        uint16_t left;
        uint16_t right;
};

static MotorDriver left_motor{LEFT_MOTOR};
static MotorDriver right_motor{RIGHT_MOTOR};
static WheelEncoder left_encoder{LEFT_ENCODER_PIN};
static WheelEncoder right_encoder{RIGHT_ENCODER_PIN};
static adc_oneshot_unit_handle_t adc;
static AoSample *samples;
static volatile bool sampling_done = false;

static void apply_to_both(float duty)
{
    left_motor.apply(duty);
    right_motor.apply(duty);
    std::printf("duty,%lld,%.4f\n", esp_timer_get_time(), duty);
}

static void start_adc()
{
    const adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_1, .clk_src = ADC_RTC_CLK_SRC_DEFAULT, .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    const adc_oneshot_chan_cfg_t channel = {.atten = ADC_ATTEN_DB_12,
                                            .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, LEFT_AO, &channel));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, RIGHT_AO, &channel));
}

// Core 1, paced by the clock: one pair of conversions every SAMPLE_PERIOD_US.
static void sample_ao(void *)
{
    int64_t next = esp_timer_get_time();
    for (int i = 0; i < SAMPLES; i++)
    {
        while (esp_timer_get_time() < next)
        {
        }
        int left = 0;
        int right = 0;
        const int64_t now = esp_timer_get_time();
        adc_oneshot_read(adc, LEFT_AO, &left);
        adc_oneshot_read(adc, RIGHT_AO, &right);
        samples[i] = {static_cast<uint32_t>(now), static_cast<uint16_t>(left),
                      static_cast<uint16_t>(right)};
        next += SAMPLE_PERIOD_US;
    }
    sampling_done = true;
    vTaskDelete(nullptr);
}

static void wait_for_start()
{
    usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
    esp_vfs_usb_serial_jtag_use_driver();
    std::printf("# encoder AO diagnostic on every step: send a line to start\n");
    char received = 0;
    while (received != '\n')
    {
        usb_serial_jtag_read_bytes(&received, 1, portMAX_DELAY);
    }
}

static void report_edges_for(int milliseconds, EdgeReporter &left, EdgeReporter &right)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (int elapsed = 0; elapsed < milliseconds; elapsed += POLL_MS)
    {
        left.report_new_edge();
        right.report_new_edge();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_MS));
    }
}

extern "C" void app_main()
{
    left_motor.start();
    right_motor.start();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    left_encoder.start();
    right_encoder.start();
    start_adc();
    samples = static_cast<AoSample *>(
        heap_caps_malloc(SAMPLES * sizeof(AoSample), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
    );
    if (samples == nullptr)
    {
        std::printf("# no memory for %d samples\n", SAMPLES);
        std::abort();
    }
    left_motor.apply(BRAKE);
    right_motor.apply(BRAKE);
    wait_for_start();

    EdgeReporter left_edges{left_encoder, 'L'};
    EdgeReporter right_edges{right_encoder, 'R'};
    for (const float direction : {1.0f, -1.0f})
    {
        for (const float duty : STEP_DUTIES)
        {
            sampling_done = false;
            apply_to_both(direction * duty);
            xTaskCreatePinnedToCore(sample_ao, "sample_ao", 4096, nullptr,
                                    configMAX_PRIORITIES - 1, nullptr, 1);
            while (!sampling_done)
            {
                report_edges_for(POLL_MS, left_edges, right_edges);
            }
            apply_to_both(BRAKE);
            report_edges_for(BRAKE_MS, left_edges, right_edges);
            for (int i = 0; i < SAMPLES; i++)
            {
                std::printf("ao,%lu,%u,%u\n", static_cast<unsigned long>(samples[i].time_us),
                            samples[i].left, samples[i].right);
            }
        }
    }
    std::printf("done\n");
}
