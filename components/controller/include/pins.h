#pragma once

// Pin map of the controller (robot-docs hardware/cabeamento.md), shared by the robot firmware and
// the bench tools in robot-firmware/tools.

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "actuation/motor_driver.h"

// DRV8871 inputs, each driver on its own LEDC timer and one channel per input.
constexpr MotorDriver::Wiring LEFT_MOTOR = {
    .timer = LEDC_TIMER_0,
    .in1 = {.pin = GPIO_NUM_4, .channel = LEDC_CHANNEL_0},
    .in2 = {.pin = GPIO_NUM_5, .channel = LEDC_CHANNEL_1},
};
constexpr MotorDriver::Wiring RIGHT_MOTOR = {
    .timer = LEDC_TIMER_1,
    .in1 = {.pin = GPIO_NUM_2, .channel = LEDC_CHANNEL_2},
    .in2 = {.pin = GPIO_NUM_42, .channel = LEDC_CHANNEL_3},
};

constexpr gpio_num_t LEFT_ENCODER_PIN = GPIO_NUM_14;
constexpr gpio_num_t RIGHT_ENCODER_PIN = GPIO_NUM_21;

constexpr gpio_num_t WATCHDOG_TEST_PIN = GPIO_NUM_11;  // V-02

// VL53L0X on I2C (ADR 020); XSHUT and GPIO1 are not wired.
constexpr gpio_num_t FRONT_RANGE_SDA_PIN = GPIO_NUM_39;
constexpr gpio_num_t FRONT_RANGE_SCL_PIN = GPIO_NUM_38;
