#pragma once

// Constants of the controller component, with the symbols of the verifications.
// Plain constexpr: each source file gets its own private copy, so the config.h of another
// component can reuse a name without conflict.

#include <chrono>
#include <cstdint>
#include <numbers>

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>

#include "pins.h"

// --- Timing (V-05, V-06) ---
constexpr std::chrono::milliseconds T_C{10};     // period of the control task
constexpr std::chrono::milliseconds T_CMD{200};  // command expiry (RNF-02)

// --- Control task ---
constexpr int TASK_CORE = 1;  // core 0 runs micro-ROS and the interrupts
constexpr int TASK_PRIORITY = configMAX_PRIORITIES - 1;  // highest
constexpr uint32_t TASK_STACK_BYTES = 4 * 1024;

// --- Motor drivers: DRV8871 on LEDC (ADR 023; pins in pins.h) ---
constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;  // the only LEDC mode of the S3
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;                 // high relative to L/R of the motor
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_11_BIT;  // finest at 20 kHz (80 MHz APB)
constexpr float DUTY_MAX = 0.48f;  // the stalled wheel gets at most the motor rated voltage

// --- Supervision (ADR 029, V-02) ---
constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT = 3 * T_C;  // a few T_C; set on the bench
// Longer than a slow reconnection of the session (robot-docs#39); set on the bench.
constexpr std::chrono::seconds WATCHDOG_FAULT_DURATION{10};

// --- Encoders (V-02; pins in pins.h) ---
constexpr float WHEEL_DIAMETER_M = 0.065f;  // D
constexpr int ENCODER_SLOTS = 20;           // N, rising edges per turn
// s (m): wheel travel between two rising edges, 10.2 mm
constexpr float WHEEL_TRAVEL_PER_EDGE = std::numbers::pi_v<float> * WHEEL_DIAMETER_M /
                                        ENCODER_SLOTS;
// No edge for T_VEL: speed zero (estimate; = s / v_min)
constexpr std::chrono::milliseconds T_VEL{150};
// Shorter than s / v_max with margin: v_max = 0.5 m/s gives 20 ms (V-02, step 9).
constexpr std::chrono::milliseconds ENCODER_MIN_EDGE_INTERVAL{15};
