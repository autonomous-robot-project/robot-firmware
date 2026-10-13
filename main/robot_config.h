#pragma once

// Constantes do firmware. Valores iniciais, a ajustar na bancada.

#include <driver/gpio.h>
#include <driver/uart.h>

// --- Núcleos ---
constexpr int CORE_UROS = 0;

// --- micro-ROS: transporte serial ---
constexpr uart_port_t UROS_UART_PORT = UART_NUM_0;
constexpr gpio_num_t UROS_UART_TX = GPIO_NUM_43;
constexpr gpio_num_t UROS_UART_RX = GPIO_NUM_44;
constexpr int UROS_UART_BAUD = 921600;
constexpr int UROS_UART_BUF_BYTES = 1024;

// --- micro-ROS: tarefa ---
constexpr int UROS_TASK_STACK_BYTES = 16000;
constexpr int UROS_TASK_PRIO = 5;
constexpr int UROS_SPIN_TIMEOUT_MS = 100;
constexpr int UROS_AGENT_PING_TIMEOUT_MS = 200;
constexpr int UROS_AGENT_PING_TRIES = 5;
constexpr int UROS_RETRY_DELAY_MS = 500;

// --- Smoke test ---
constexpr int SMOKE_PUB_PERIOD_MS = 1000;
constexpr int SMOKE_CORE_FACTOR = 10000;
