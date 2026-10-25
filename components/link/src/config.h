#pragma once

// Constants of the link component. Initial values, adjusted on the bench.

#include <driver/gpio.h>
#include <driver/uart.h>

// --- UART (ADR 043) ---
constexpr uart_port_t UART_PORT = UART_NUM_1;
constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_17;  // provisional, issue #38
constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_18;  // provisional, issue #38
constexpr int UART_BAUD = 921600;
constexpr int UART_RX_BUFFER_BYTES = 1024;  // above the XRCE-DDS serial MTU (512 B)

// --- micro-ROS task ---
constexpr int TASK_CORE = 0;  // core 1 is reserved for the control task
constexpr int TASK_PRIORITY = 5;
constexpr int TASK_STACK_BYTES = 16 * 1024;

// --- Agent liveness ---
constexpr int AGENT_PING_TIMEOUT_MS = 100;
constexpr int AGENT_PING_PERIOD_MS = 1000;
constexpr int AGENT_RETRY_PERIOD_MS = 500;

constexpr const char *NODE_NAME = "controller";
