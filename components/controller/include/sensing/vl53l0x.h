#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

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

        // One ranging: the distance and the device range status of the ST API (bits 6 to 3 of
        // RESULT_RANGE_STATUS), kept raw; robot-docs nota 0015 classifies them.
        struct Measurement
        {
            public:
                uint16_t distance_mm;
                uint8_t device_status;
        };

        explicit VL53L0X(const Wiring &wiring);
        VL53L0X(const VL53L0X &) = delete;
        VL53L0X &operator=(const VL53L0X &) = delete;

        // Creates the I2C bus and the device. Called once, at boot, by the task that reads it.
        void start();

        // Brings the sensor from reset to ready for ranging, as the Pololu init(): data init,
        // static init (reference SPADs and the ST tuning) and reference calibration. Blocks the
        // caller between polls, for tens of ms. Also the recovery after N_I2C failures (D4).
        esp_err_t initialize();

        // Starts the continuous back-to-back mode with the timing budget T_BUDGET_VLX: the sensor
        // ranges again as soon as it ends (robot-docs nota 0015).
        esp_err_t start_continuous();

        // Waits, polling every T_POLL for at most T_WAIT, for the next ranging of the continuous
        // mode, reads it and clears its interrupt.
        esp_err_t read_measurement(Measurement &measurement);

        // Releases a slave that holds SDA low, with clock pulses on SCL (robot-docs nota 0015).
        esp_err_t reset_bus();

    private:
        struct RegisterWrite
        {
            public:
                uint8_t reg;
                uint8_t value;
        };

        // Reference SPADs (single photon avalanche diodes) stored in the sensor at the factory.
        struct SpadInfo
        {
            public:
                uint8_t count;
                bool is_aperture;
        };

        // The phases of initialize(), named after the ST API functions they port.
        esp_err_t data_init();
        esp_err_t static_init();
        esp_err_t read_spad_info(SpadInfo &info);
        esp_err_t set_reference_spads(const SpadInfo &info);
        esp_err_t configure_interrupt();
        esp_err_t reference_calibration();
        esp_err_t single_reference_calibration(uint8_t vhv_init);
        esp_err_t set_timing_budget(std::chrono::microseconds budget);
        esp_err_t enter_private_page();
        esp_err_t leave_private_page();
        esp_err_t load_tuning_settings();

        // Polls `reg` every T_POLL until a bit of `mask` is set, for at most T_WAIT.
        esp_err_t wait_until_set(uint8_t reg, uint8_t mask);

        esp_err_t write_registers(std::span<const RegisterWrite> writes);
        esp_err_t set_bits(uint8_t reg, uint8_t mask);
        esp_err_t clear_bits(uint8_t reg, uint8_t mask);
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
        // Read in the data init and written back to start each ranging (ST API).
        uint8_t stop_variable_ = 0;
};
