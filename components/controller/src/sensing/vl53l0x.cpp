// Criticality: T2, product (robot-docs codigo.md).
#include "sensing/vl53l0x.h"

#include <algorithm>
#include <array>

#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

static constexpr int TIMEOUT_MS = static_cast<int>(T_I2C.count());
static constexpr uint8_t GLITCH_FILTER_CYCLES = 7;  // the default of the IDF examples
// Longest block written: the 6 bytes of the reference SPAD map, after the register address.
static constexpr size_t MAX_WRITE_BLOCK = 6;

static const char *TAG = "vl53l0x";

// Registers named in the ST API (vl53l0x_device.h).
static constexpr uint8_t SYSRANGE_START = 0x00;
static constexpr uint8_t SYSTEM_SEQUENCE_CONFIG = 0x01;
static constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
static constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
static constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
static constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
static constexpr uint8_t DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
static constexpr uint8_t DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
static constexpr uint8_t MSRC_CONFIG_CONTROL = 0x60;
static constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
static constexpr uint8_t VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89;
static constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
static constexpr uint8_t GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
static constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;

// Values of the ST API and the datasheet.
static constexpr uint8_t MODEL_ID = 0xEE;
static constexpr uint8_t IO_2V8 = 0x01;  // the module regulates the sensor to 2.8 V
// Signal rate limit checks of the MSRC (bit 1) and of the pre-range (bit 4), disabled.
static constexpr uint8_t MSRC_AND_PRE_RANGE_LIMIT_CHECKS = 0x12;
// Final range signal rate limit: 0.25 MCPS in fixed point Q9.7, the ST default.
static constexpr uint16_t SIGNAL_RATE_LIMIT_Q9_7 = static_cast<uint16_t>(0.25f * (1 << 7));
static constexpr uint8_t ALL_SEQUENCE_STEPS = 0xFF;
// Sequence without the MSRC and TCC steps, the Pololu default.
static constexpr uint8_t RANGING_SEQUENCE_STEPS = 0xE8;
static constexpr uint8_t VHV_CALIBRATION_STEP = 0x01;
static constexpr uint8_t PHASE_CALIBRATION_STEP = 0x02;
static constexpr uint8_t VHV_CALIBRATION_INIT = 0x40;
static constexpr uint8_t PHASE_CALIBRATION_INIT = 0x00;
static constexpr uint8_t START_SINGLE_RANGING = 0x01;
static constexpr uint8_t STOP_RANGING = 0x00;
static constexpr uint8_t NEW_SAMPLE_READY = 0x04;
static constexpr uint8_t INTERRUPT_ACTIVE_HIGH = 0x10;
static constexpr uint8_t INTERRUPT_PENDING = 0x07;
static constexpr uint8_t CLEAR_INTERRUPT = 0x01;

// Reference SPAD map: 48 SPADs, the aperture ones from the 12th (ST API).
static constexpr size_t SPAD_MAP_BYTES = 6;
static constexpr uint8_t SPAD_COUNT = 48;
static constexpr uint8_t FIRST_APERTURE_SPAD = 12;
static constexpr uint8_t REQUESTED_REFERENCE_SPADS = 0x2C;
static constexpr uint8_t REFERENCE_START_SELECT = 0xB4;
static constexpr uint8_t SPAD_COUNT_MASK = 0x7F;
static constexpr uint8_t SPAD_APERTURE_BIT = 0x80;

static constexpr int MAX_POLLS = static_cast<int>(T_WAIT / T_POLL);

static constexpr uint8_t STOP_VARIABLE = 0x91;
static constexpr uint8_t I2C_STANDARD_MODE = 0x88;

VL53L0X::VL53L0X(const Wiring &wiring) : wiring_(wiring)
{
}

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

esp_err_t VL53L0X::reset_bus()
{
    return i2c_master_bus_reset(bus_);
}

// Each check below returns the error and logs the step that failed (esp_check.h). The
// expression is always evaluated: the silent-check option of the IDF only drops the message.

esp_err_t VL53L0X::initialize()
{
    uint8_t model = 0;
    ESP_RETURN_ON_ERROR(read_register(IDENTIFICATION_MODEL_ID, model), TAG, "model id");
    ESP_RETURN_ON_FALSE(model == MODEL_ID, ESP_ERR_NOT_FOUND, TAG, "model id 0x%02x", model);

    ESP_RETURN_ON_ERROR(data_init(), TAG, "data init");
    ESP_RETURN_ON_ERROR(static_init(), TAG, "static init");
    ESP_RETURN_ON_ERROR(reference_calibration(), TAG, "reference calibration");
    return ESP_OK;
}

esp_err_t VL53L0X::data_init()
{
    ESP_RETURN_ON_ERROR(set_bits(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, IO_2V8), TAG, "2.8 V I/O");
    ESP_RETURN_ON_ERROR(write_register(I2C_STANDARD_MODE, 0x00), TAG, "standard mode");

    ESP_RETURN_ON_ERROR(enter_private_page(), TAG, "enter page");
    ESP_RETURN_ON_ERROR(read_register(STOP_VARIABLE, stop_variable_), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(leave_private_page(), TAG, "leave page");

    ESP_RETURN_ON_ERROR(
        set_bits(MSRC_CONFIG_CONTROL, MSRC_AND_PRE_RANGE_LIMIT_CHECKS), TAG, "limit checks"
    );
    ESP_RETURN_ON_ERROR(
        write_register_16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, SIGNAL_RATE_LIMIT_Q9_7),
        TAG,
        "signal rate limit"
    );
    return write_register(SYSTEM_SEQUENCE_CONFIG, ALL_SEQUENCE_STEPS);
}

esp_err_t VL53L0X::static_init()
{
    SpadInfo spads{};
    ESP_RETURN_ON_ERROR(read_spad_info(spads), TAG, "spad info");
    ESP_RETURN_ON_ERROR(set_reference_spads(spads), TAG, "reference spads");
    ESP_RETURN_ON_ERROR(load_tuning_settings(), TAG, "tuning settings");
    ESP_RETURN_ON_ERROR(configure_interrupt(), TAG, "interrupt");
    return write_register(SYSTEM_SEQUENCE_CONFIG, RANGING_SEQUENCE_STEPS);
}

esp_err_t VL53L0X::read_spad_info(SpadInfo &info)
{
    static constexpr RegisterWrite REQUEST_SPAD_INFO[] = {
        {0xFF, 0x07},
        {0x81, 0x01},
        {0x80, 0x01},
        {0x94, 0x6B},
        {0x83, 0x00},
    };
    static constexpr RegisterWrite END_SPAD_INFO[] = {{0x81, 0x00}, {0xFF, 0x06}};
    static constexpr uint8_t SPAD_INFO_READY = 0x83;
    static constexpr uint8_t SPAD_INFO = 0x92;
    static constexpr uint8_t SPAD_INFO_ENABLE = 0x04;

    ESP_RETURN_ON_ERROR(enter_private_page(), TAG, "enter page");
    ESP_RETURN_ON_ERROR(write_register(0xFF, 0x06), TAG, "page 6");
    ESP_RETURN_ON_ERROR(set_bits(SPAD_INFO_READY, SPAD_INFO_ENABLE), TAG, "enable");
    ESP_RETURN_ON_ERROR(write_registers(REQUEST_SPAD_INFO), TAG, "request");
    ESP_RETURN_ON_ERROR(wait_until_set(SPAD_INFO_READY, 0xFF), TAG, "wait");
    ESP_RETURN_ON_ERROR(write_register(SPAD_INFO_READY, 0x01), TAG, "acknowledge");

    uint8_t packed = 0;
    ESP_RETURN_ON_ERROR(read_register(SPAD_INFO, packed), TAG, "read");
    info.count = packed & SPAD_COUNT_MASK;
    info.is_aperture = (packed & SPAD_APERTURE_BIT) != 0;

    ESP_RETURN_ON_ERROR(write_registers(END_SPAD_INFO), TAG, "end");
    ESP_RETURN_ON_ERROR(clear_bits(SPAD_INFO_READY, SPAD_INFO_ENABLE), TAG, "disable");
    ESP_RETURN_ON_ERROR(write_register(0xFF, 0x01), TAG, "page 1");
    return leave_private_page();
}

// Enables the first `info.count` good SPADs of the factory map, from the first aperture SPAD
// when the type is aperture (ST API VL53L0X_set_reference_spads, assuming a valid NVM).
esp_err_t VL53L0X::set_reference_spads(const SpadInfo &info)
{
    std::array<uint8_t, SPAD_MAP_BYTES> map{};
    ESP_RETURN_ON_ERROR(
        read_block(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map.data(), map.size()), TAG, "read map"
    );

    static constexpr RegisterWrite SELECT_REFERENCE_SPADS[] = {
        {0xFF, 0x01},
        {DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00},
        {DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, REQUESTED_REFERENCE_SPADS},
        {0xFF, 0x00},
        {GLOBAL_CONFIG_REF_EN_START_SELECT, REFERENCE_START_SELECT},
    };
    ESP_RETURN_ON_ERROR(write_registers(SELECT_REFERENCE_SPADS), TAG, "select");

    const uint8_t first = info.is_aperture ? FIRST_APERTURE_SPAD : 0;
    uint8_t enabled = 0;
    for (uint8_t spad = 0; spad < SPAD_COUNT; ++spad)
    {
        uint8_t &byte = map[spad / 8];
        const uint8_t bit = 1 << (spad % 8);
        const bool kept = spad >= first && enabled < info.count;
        if (!kept)
        {
            byte &= ~bit;
        }
        else if ((byte & bit) != 0)
        {
            ++enabled;
        }
    }
    return write_block(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map.data(), map.size());
}

// Interrupt on a new sample, active low; the task polls it, since GPIO1 is not wired.
esp_err_t VL53L0X::configure_interrupt()
{
    ESP_RETURN_ON_ERROR(
        write_register(SYSTEM_INTERRUPT_CONFIG_GPIO, NEW_SAMPLE_READY), TAG, "new sample"
    );
    ESP_RETURN_ON_ERROR(clear_bits(GPIO_HV_MUX_ACTIVE_HIGH, INTERRUPT_ACTIVE_HIGH), TAG, "low");
    return write_register(SYSTEM_INTERRUPT_CLEAR, CLEAR_INTERRUPT);
}

esp_err_t VL53L0X::reference_calibration()
{
    ESP_RETURN_ON_ERROR(write_register(SYSTEM_SEQUENCE_CONFIG, VHV_CALIBRATION_STEP), TAG, "vhv");
    ESP_RETURN_ON_ERROR(single_reference_calibration(VHV_CALIBRATION_INIT), TAG, "vhv");

    ESP_RETURN_ON_ERROR(
        write_register(SYSTEM_SEQUENCE_CONFIG, PHASE_CALIBRATION_STEP), TAG, "phase"
    );
    ESP_RETURN_ON_ERROR(single_reference_calibration(PHASE_CALIBRATION_INIT), TAG, "phase");

    return write_register(SYSTEM_SEQUENCE_CONFIG, RANGING_SEQUENCE_STEPS);
}

esp_err_t VL53L0X::single_reference_calibration(uint8_t vhv_init)
{
    const uint8_t start = START_SINGLE_RANGING | vhv_init;
    ESP_RETURN_ON_ERROR(write_register(SYSRANGE_START, start), TAG, "start");
    ESP_RETURN_ON_ERROR(wait_until_set(RESULT_INTERRUPT_STATUS, INTERRUPT_PENDING), TAG, "wait");
    ESP_RETURN_ON_ERROR(write_register(SYSTEM_INTERRUPT_CLEAR, CLEAR_INTERRUPT), TAG, "clear");
    return write_register(SYSRANGE_START, STOP_RANGING);
}

// Undocumented sequences of the ST API, as ported by Pololu. Register 0xFF selects the page.
esp_err_t VL53L0X::enter_private_page()
{
    static constexpr RegisterWrite ENTER[] = {{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}};
    return write_registers(ENTER);
}

esp_err_t VL53L0X::leave_private_page()
{
    static constexpr RegisterWrite LEAVE[] = {{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}};
    return write_registers(LEAVE);
}

// DefaultTuningSettings of the ST API (vl53l0x_tuning.h), as listed by Pololu.
esp_err_t VL53L0X::load_tuning_settings()
{
    static constexpr RegisterWrite TUNING_SETTINGS[] = {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
        {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
        {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
        {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
        {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
        {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
        {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
        {0xFF, 0x00}, {0x80, 0x00},
    };
    return write_registers(TUNING_SETTINGS);
}

// Register and mask are both bytes of the ST API; a distinct type would only wrap them.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
esp_err_t VL53L0X::wait_until_set(uint8_t reg, uint8_t mask)
{
    for (int poll = 0; poll < MAX_POLLS; ++poll)
    {
        uint8_t value = 0;
        const esp_err_t read = read_register(reg, value);
        if (read != ESP_OK)
        {
            return read;
        }
        if ((value & mask) != 0)
        {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(T_POLL.count()));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t VL53L0X::write_registers(std::span<const RegisterWrite> writes)
{
    for (const RegisterWrite &write : writes)
    {
        const esp_err_t written = write_register(write.reg, write.value);
        if (written != ESP_OK)
        {
            return written;
        }
    }
    return ESP_OK;
}

esp_err_t VL53L0X::set_bits(uint8_t reg, uint8_t mask)
{
    uint8_t value = 0;
    const esp_err_t read = read_register(reg, value);
    if (read != ESP_OK)
    {
        return read;
    }
    return write_register(reg, value | mask);
}

esp_err_t VL53L0X::clear_bits(uint8_t reg, uint8_t mask)
{
    uint8_t value = 0;
    const esp_err_t read = read_register(reg, value);
    if (read != ESP_OK)
    {
        return read;
    }
    return write_register(reg, value & ~mask);
}

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
