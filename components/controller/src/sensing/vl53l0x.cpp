// Criticality: T2, product (robot-docs codigo.md).
#include "sensing/vl53l0x.h"

#include <algorithm>
#include <array>

#include "config.h"

static constexpr int TIMEOUT_MS = static_cast<int>(T_I2C.count());
static constexpr uint8_t GLITCH_FILTER_CYCLES = 7;  // the default of the IDF examples
// Longest block written: the 6 bytes of the reference SPAD map, after the register address.
static constexpr size_t MAX_WRITE_BLOCK = 6;

VL53L0X::VL53L0X(const Wiring &wiring) : wiring_(wiring) {}

void VL53L0X::start()
{
    // The GY-530 type module has its own pull-ups (to confirm on the bench).
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = wiring_.sda,
        .scl_io_num = wiring_.scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = GLITCH_FILTER_CYCLES,
        .intr_priority = 0,      // the default priority
        .trans_queue_depth = 0,  // synchronous transactions
        .flags = {.enable_internal_pullup = false},
    };
    const esp_err_t bus_created = i2c_new_master_bus(&bus_config, &bus_);
    ESP_ERROR_CHECK(bus_created);

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
        .scl_wait_us = 0,  // the default clock stretching
        .flags = {.disable_ack_check = false},
    };
    const esp_err_t device_added = i2c_master_bus_add_device(bus_, &device_config, &device_);
    ESP_ERROR_CHECK(device_added);
}

esp_err_t VL53L0X::reset_bus() { return i2c_master_bus_reset(bus_); }

// Multi-byte registers are big-endian (ST API).

esp_err_t VL53L0X::write_register(uint8_t reg, uint8_t value)
{
    const std::array<uint8_t, 2> frame = {reg, value};
    return i2c_master_transmit(device_, frame.data(), frame.size(), TIMEOUT_MS);
}

esp_err_t VL53L0X::write_register_16(uint8_t reg, uint16_t value)
{
    const std::array<uint8_t, 3> frame = {
        reg,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value),
    };
    return i2c_master_transmit(device_, frame.data(), frame.size(), TIMEOUT_MS);
}

esp_err_t VL53L0X::read_register(uint8_t reg, uint8_t &value)
{
    return read_block(reg, &value, 1);
}

esp_err_t VL53L0X::read_register_16(uint8_t reg, uint16_t &value)
{
    std::array<uint8_t, 2> bytes{};
    const esp_err_t read = read_block(reg, bytes.data(), bytes.size());
    value = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
    return read;
}

esp_err_t VL53L0X::write_block(uint8_t first, const uint8_t *values, size_t size)
{
    if (size > MAX_WRITE_BLOCK)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    std::array<uint8_t, 1 + MAX_WRITE_BLOCK> frame{};
    frame[0] = first;
    std::copy_n(values, size, frame.begin() + 1);
    return i2c_master_transmit(device_, frame.data(), 1 + size, TIMEOUT_MS);
}

esp_err_t VL53L0X::read_block(uint8_t first, uint8_t *values, size_t size)
{
    return i2c_master_transmit_receive(device_, &first, 1, values, size, TIMEOUT_MS);
}
