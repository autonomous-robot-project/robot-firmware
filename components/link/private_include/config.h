#pragma once

// Constants of the link component. Initial values, adjusted on the bench.
// Plain constexpr: each source file gets its own private copy, so the config.h of another
// component can reuse a name without conflict.

#include <chrono>
#include <cstdint>

#include "uart_transport.h"

// --- UART (ADR 043) ---
constexpr UartTransport::Config UART_CONFIG = {
    .port = 1,     // UART1
    .tx_pin = 41,  // robot-docs hardware/cabeamento.md
    .rx_pin = 40,
    .baud = 921600,
    .rx_buffer_bytes = 2048,  // above the XRCE-DDS MTU (1024 B, app-colcon.meta, ADR 043)
};

// --- micro-ROS task ---
constexpr int TASK_CORE = 0;  // core 1 is reserved for the control task
constexpr int TASK_PRIORITY = 5;
constexpr uint32_t TASK_STACK_BYTES = 16 * 1024;

// --- Agent liveness ---
constexpr std::chrono::milliseconds AGENT_PING_TIMEOUT{100};
constexpr std::chrono::milliseconds AGENT_PING_PERIOD{1000};
constexpr std::chrono::milliseconds AGENT_RETRY_PERIOD{500};

// --- Topics (ADR 025) ---
constexpr const char *NODE_NAME = "controller";
constexpr const char *COMMAND_TOPIC = "/controller/command";
constexpr const char *STATE_TOPIC = "/controller/state";
constexpr const char *INSTRUMENTATION_TOPIC = "/controller/instrumentation";  // ADR 028
constexpr const char *ODOMETRY_TOPIC = "/controller/odometry";  // ADR 048
constexpr const char *ODOMETRY_FRAME = "odom";
constexpr const char *ODOMETRY_CHILD_FRAME = "base_link";

// --- Exchange with the controller ---
constexpr std::chrono::milliseconds STATE_PERIOD{10};             // 100 Hz = 1/T_C (ADR 025)
constexpr std::chrono::milliseconds INSTRUMENTATION_PERIOD{100};  // 10 Hz (ADR 025)
constexpr std::chrono::milliseconds ODOMETRY_PERIOD{50};          // 20 Hz (ADR 025)

// --- Clock synchronization with the agent (ADR 027) ---
constexpr std::chrono::milliseconds CLOCK_SYNC_TIMEOUT{100};
// Provisional, until the drift between the clocks is measured (V-04, robot-docs#35).
constexpr std::chrono::milliseconds CLOCK_SYNC_PERIOD{10000};
