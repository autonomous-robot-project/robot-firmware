#pragma once

#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

// VL53L0X time-of-flight sensor on its own I2C bus: a lean port of the Pololu library (MIT),
// itself based on the ST API (robot-docs nota 0015). Every transaction has the timeout T_I2C and
// returns its error to the caller, which decides the recovery (ADR 020).
// Criticality: T2, product (robot-docs codigo.md).
class VL53L0X
{
    public:
        // Where the sensor is wired.
        struct Wiring
        {
            public:
                gpio_num_t sda;
                gpio_num_t scl;
        };

        explicit VL53L0X(const Wiring &wiring);
        VL53L0X(const VL53L0X &) = delete;
        VL53L0X &operator=(const VL53L0X &) = delete;

        // Creates the I2C bus and the device. Called once, at boot, by the task that reads it.
        void start();

        // Releases a slave that holds SDA low, with clock pulses on SCL (robot-docs nota 0015).
        esp_err_t reset_bus();

    private:
        esp_err_t write_register(uint8_t reg, uint8_t value);
        esp_err_t write_register_16(uint8_t reg, uint16_t value);
        esp_err_t read_register(uint8_t reg, uint8_t &value);
        esp_err_t read_register_16(uint8_t reg, uint16_t &value);

        // Write or read `size` consecutive registers from `first`, in one transaction.
        esp_err_t write_block(uint8_t first, const uint8_t *values, size_t size);
        esp_err_t read_block(uint8_t first, uint8_t *values, size_t size);

        Wiring wiring_;
        i2c_master_bus_handle_t bus_ = nullptr;
        i2c_master_dev_handle_t device_ = nullptr;
};
