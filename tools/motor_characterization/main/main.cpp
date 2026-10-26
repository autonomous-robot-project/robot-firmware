// Bench tool: open-loop characterization of the two motors, with the wheels suspended.
// Applies the DutySchedule to both motors and prints CSV on the USB console:
//   duty,<t_us>,<duty>         every DUTY_PERIOD
//   edge,<t_us>,<L|R>,<count>  every encoder edge
//   done                       at the end, with both wheels braked
// Nothing turns until the host sends a line: tools/motor_characterization/characterize.py.

#include <chrono>
#include <cstdio>
#include <optional>

#include <driver/gpio.h>
#include <driver/usb_serial_jtag.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_vfs_dev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "actuation/motor_driver.h"
#include "duty_schedule.h"
#include "edge_reporter.h"
#include "pins.h"
#include "sensing/wheel_encoder.h"

using std::chrono::microseconds;
using std::chrono::milliseconds;

constexpr milliseconds POLL_PERIOD{2};   // shorter than the closest edges (15 ms)
constexpr milliseconds DUTY_PERIOD{10};  // T_C of the controller
constexpr int POLLS_PER_DUTY_UPDATE = DUTY_PERIOD / POLL_PERIOD;
constexpr float BRAKE = 0.0f;

static MotorDriver left_motor{LEFT_MOTOR};
static MotorDriver right_motor{RIGHT_MOTOR};
static WheelEncoder left_encoder{LEFT_ENCODER_PIN};
static WheelEncoder right_encoder{RIGHT_ENCODER_PIN};

static microseconds now()
{
    return microseconds{esp_timer_get_time()};
}

static void start_hardware()
{
    left_motor.start();
    right_motor.start();
    constexpr int default_flags = 0;
    const esp_err_t installed = gpio_install_isr_service(default_flags);
    ESP_ERROR_CHECK(installed);
    left_encoder.start();
    right_encoder.start();
}

// Blocks until the host sends a line on the USB console.
static void wait_for_start()
{
    usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t installed = usb_serial_jtag_driver_install(&config);
    ESP_ERROR_CHECK(installed);
    esp_vfs_usb_serial_jtag_use_driver();
    std::printf("# motor_characterization: wheels suspended? send a line to start\n");
    std::printf("# Each step ends in the full brake: watch the first ones, at low speed,\n");
    std::printf("# and cut the motor supply if the gearbox shows trouble (ADR 023).\n");
    char received = 0;
    while (received != '\n')
    {
        usb_serial_jtag_read_bytes(&received, 1, portMAX_DELAY);
    }
}

static void apply_to_both(float duty)
{
    left_motor.apply(duty);
    right_motor.apply(duty);
}

// Applies and prints the duty of the schedule at this instant. False after the end of the
// schedule.
static bool apply_scheduled_duty(const DutySchedule &schedule, microseconds start)
{
    const microseconds instant = now();
    const std::optional<float> duty = schedule.duty_at(
        std::chrono::duration_cast<milliseconds>(instant - start)
    );
    if (!duty)
    {
        return false;
    }
    apply_to_both(*duty);
    std::printf("duty,%lld,%.4f\n", instant.count(), *duty);
    return true;
}

extern "C" void app_main()
{
    start_hardware();
    apply_to_both(BRAKE);
    wait_for_start();

    const DutySchedule schedule;
    EdgeReporter left_edges{left_encoder, 'L'};
    EdgeReporter right_edges{right_encoder, 'R'};
    const microseconds start = now();
    TickType_t last_wake = xTaskGetTickCount();
    for (int poll = 0;; poll++)
    {
        const bool duty_update_due = poll % POLLS_PER_DUTY_UPDATE == 0;
        if (duty_update_due && !apply_scheduled_duty(schedule, start))
        {
            break;
        }
        left_edges.report_new_edge();
        right_edges.report_new_edge();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD.count()));
    }
    apply_to_both(BRAKE);
    std::printf("done\n");
}
