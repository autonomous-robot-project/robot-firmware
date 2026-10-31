#pragma once

// Constants of the controller component that depend on the hardware, with the symbols of the
// verifications. The constants of the control are in control_config.h.
// Plain constexpr: each source file gets its own private copy, so the config.h of another
// component can reuse a name without conflict.

#include <chrono>
#include <cstdint>

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>

#include "control_config.h"
#include "pins.h"

// --- Command (V-06) ---
constexpr std::chrono::milliseconds T_CMD{200};  // command expiry (RNF-02)

// --- Control task ---
constexpr int TASK_CORE = 1;  // core 0 runs micro-ROS and the interrupts
constexpr int TASK_PRIORITY = configMAX_PRIORITIES - 1;  // highest
constexpr uint32_t TASK_STACK_BYTES = 4 * 1024;

// --- Motor drivers: DRV8871 on LEDC (ADR 023; pins in pins.h) ---
constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;  // the only LEDC mode of the S3
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;                 // high relative to L/R of the motor
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_11_BIT;  // finest at 20 kHz (80 MHz APB)

// --- Supervision (ADR 029, V-02) ---
constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT = 3 * T_C;  // a few T_C; set on the bench
// Longer than a slow reconnection of the session (robot-docs#39); set on the bench.
constexpr std::chrono::seconds WATCHDOG_FAULT_DURATION{10};

// --- Encoders (V-02; pins in pins.h) ---
// The comparator chatters at both transitions of a slot (robot-docs nota 0010): a level counts
// after ENCODER_STABLE_SAMPLES samples, 2 ms, longer than the chatter and shorter than half a slot
// at v_max (s / (2 v_max) = 10 ms at 0.5 m/s).
constexpr std::chrono::microseconds ENCODER_SAMPLE_PERIOD{500};
constexpr int ENCODER_STABLE_SAMPLES = 4;

// --- Front range sensor: VL53L0X on I2C (ADR 020, robot-docs nota 0015; pins in pins.h) ---
constexpr uint16_t VL53L0X_ADDRESS = 0x29;     // 7-bit address after reset (datasheet)
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;  // fast mode, the maximum of the VL53L0X (datasheet)
// T_I2C: timeout of one transaction. The longest one, 12 bytes at 400 kHz, takes ~0.3 ms.
constexpr std::chrono::milliseconds T_I2C{10};
// T_POLL_VLX: period of the polls of the sensor status, the task blocked in between (GPIO1 is not
// wired). It delays the stamp of a measurement by at most T_POLL_VLX, against the ~33 ms between
// them.
constexpr std::chrono::milliseconds T_POLL_VLX{2};
// T_WAIT_VLX: longest wait for the sensor, as in the Pololu examples; above one ranging (~33 ms).
constexpr std::chrono::milliseconds T_WAIT_VLX{500};
// T_BUDGET_VLX: timing budget of one ranging, the ~33 ms default of the ST API (robot-docs nota
// 0015).
constexpr std::chrono::microseconds T_BUDGET_VLX{33000};
// Range offset of this module (robot-docs nota 0015). Its factory register held +53.5 mm (0x00D6),
// and with it the sensor still read 41.1 mm long at d_stop, by the line through two measured
// points. The value is absolute: the sensor keeps its power across a restart of the ESP32, so
// adding to the register at each initialization would apply the correction twice.
constexpr int32_t FACTORY_OFFSET_VLX_UM = 53500;
constexpr int32_t BIAS_AT_D_STOP_VLX_UM = 41100;
constexpr int32_t RANGE_OFFSET_VLX_UM = FACTORY_OFFSET_VLX_UM - BIAS_AT_D_STOP_VLX_UM;

// --- Sensor task (ADR 020; recovery in robot-docs nota 0015) ---
constexpr int SENSOR_TASK_CORE = 0;      // with micro-ROS, off the core of the control task
constexpr int SENSOR_TASK_PRIORITY = 6;  // above the micro-ROS task (5, link config.h)
constexpr uint32_t SENSOR_TASK_STACK_BYTES = 4 * 1024;
// N_I2C: consecutive failed readings before a new initialization. Tolerates an isolated glitch.
constexpr int N_I2C = 3;
// T_RETRY: wait before each new initialization. Logs a missing sensor at 2 Hz; the safety does
// not depend on it, since the ranging ages meanwhile (ADR 021).
constexpr std::chrono::milliseconds T_RETRY{500};
