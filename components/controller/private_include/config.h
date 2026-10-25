#pragma once

// Constants of the controller component, with the symbols of the verifications.
// Plain constexpr: each source file gets its own private copy, so the config.h of another
// component can reuse a name without conflict.

#include <chrono>
#include <cstdint>

#include <freertos/FreeRTOS.h>

// --- Timing (V-05, V-06) ---
constexpr std::chrono::milliseconds T_C{10};     // period of the control task
constexpr std::chrono::milliseconds T_CMD{200};  // command expiry (RNF-02)

// --- Control task ---
constexpr int TASK_CORE = 1;  // core 0 runs micro-ROS and the interrupts
constexpr int TASK_PRIORITY = configMAX_PRIORITIES - 1;  // highest
constexpr uint32_t TASK_STACK_BYTES = 4 * 1024;
