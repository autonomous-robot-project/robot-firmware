#include "uart_transport.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

static const char *TAG = "uart_transport";

static uart_port_t port_of(const UartTransport::Config &config)
{
    return static_cast<uart_port_t>(config.port);
}

void UartTransport::register_with_micro_ros()
{
    // Framing on: a serial stream has no message boundaries, so XRCE-DDS delimits each message.
    constexpr bool framing = true;
    rmw_uros_set_custom_transport(
        framing,
        this,
        &UartTransport::open,
        &UartTransport::close,
        &UartTransport::write,
        &UartTransport::read
    );
}

const UartTransport::Config &UartTransport::config_of(uxrCustomTransport *transport)
{
    return static_cast<const UartTransport *>(transport->args)->config_;
}

bool UartTransport::open(uxrCustomTransport *transport)
{
    const Config &config = config_of(transport);
    const uart_port_t port = port_of(config);

    // No TX buffer: a write returns once the bytes are in the hardware FIFO, so its duration
    // is bounded by the message size (about 11 us per byte at 921600 baud).
    constexpr int tx_buffer_bytes = 0;
    const esp_err_t installed = uart_driver_install(
        port, config.rx_buffer_bytes, tx_buffer_bytes, 0, nullptr, 0
    );
    if (installed != ESP_OK)
    {
        ESP_LOGE(TAG, "driver install failed");
        return false;
    }

    const uart_config_t uart_config = {
        .baud_rate = config.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,  // unused without flow control
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };
    const esp_err_t configured = uart_param_config(port, &uart_config);
    if (configured != ESP_OK)
    {
        ESP_LOGE(TAG, "parameter config failed");
        return false;
    }

    const esp_err_t pins_set = uart_set_pin(
        port, config.tx_pin, config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
    );
    if (pins_set != ESP_OK)
    {
        ESP_LOGE(TAG, "pin config failed");
        return false;
    }
    return true;
}

bool UartTransport::close(uxrCustomTransport *transport)
{
    return uart_driver_delete(port_of(config_of(transport))) == ESP_OK;
}

size_t UartTransport::write(
    uxrCustomTransport *transport, const uint8_t *buffer, size_t length, uint8_t *error
)
{
    const int written = uart_write_bytes(port_of(config_of(transport)), buffer, length);
    if (written < 0)
    {
        *error = 1;
        return 0;
    }
    return static_cast<size_t>(written);
}

size_t UartTransport::read(
    uxrCustomTransport *transport, uint8_t *buffer, size_t length, int timeout_ms, uint8_t *error
)
{
    const int read = uart_read_bytes(
        port_of(config_of(transport)), buffer, length, pdMS_TO_TICKS(timeout_ms)
    );
    if (read < 0)
    {
        *error = 1;
        return 0;
    }
    return static_cast<size_t>(read);
}
