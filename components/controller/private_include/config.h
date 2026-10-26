#pragma once

// Constants of the controller component, with the symbols of the verifications.
// Plain constexpr: each source file gets its own private copy, so the config.h of another
// component can reuse a name without conflict.

#include <chrono>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>

#include "actuation/motor_driver.h"

// --- Timing (V-05, V-06) ---
constexpr std::chrono::milliseconds T_C{10};     // period of the control task
constexpr std::chrono::milliseconds T_CMD{200};  // command expiry (RNF-02)

// --- Control task ---
constexpr int TASK_CORE = 1;  // core 0 runs micro-ROS and the interrupts
constexpr int TASK_PRIORITY = configMAX_PRIORITIES - 1;  // highest
constexpr uint32_t TASK_STACK_BYTES = 4 * 1024;

// --- Motor drivers: DRV8871 on LEDC (ADR 023; pins in robot-docs hardware/cabeamento.md) ---
constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;  // the only LEDC mode of the S3
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;  // high relative to L/R of the motor
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_11_BIT;  // finest at 20 kHz (80 MHz APB)
constexpr float DUTY_MAX = 0.48f;  // the stalled wheel gets at most the motor rated voltage
constexpr MotorDriver::Wiring LEFT_MOTOR = {
    .timer = LEDC_TIMER_0,
    .in1 = {.pin = GPIO_NUM_11, .channel = LEDC_CHANNEL_0},
    .in2 = {.pin = GPIO_NUM_12, .channel = LEDC_CHANNEL_1},
};
constexpr MotorDriver::Wiring RIGHT_MOTOR = {
    .timer = LEDC_TIMER_1,
    .in1 = {.pin = GPIO_NUM_13, .channel = LEDC_CHANNEL_2},
    .in2 = {.pin = GPIO_NUM_14, .channel = LEDC_CHANNEL_3},
};

// --- Supervision (ADR 029, V-02) ---
constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT = 3 * T_C;  // a few T_C; set on the bench
// Longer than a slow reconnection of the session (robot-docs#39); set on the bench.
constexpr std::chrono::seconds WATCHDOG_FAULT_DURATION{10};
constexpr gpio_num_t WATCHDOG_TEST_PIN = GPIO_NUM_48;  // robot-docs hardware/cabeamento.md
