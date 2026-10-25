#include "transport.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "config.h"

namespace {

bool uart_open(uxrCustomTransport *)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // No TX buffer: a write returns once the bytes are in the hardware FIFO, so its duration
    // is bounded by the message size (about 11 us per byte at 921600 baud).
    return uart_driver_install(UART_PORT, UART_RX_BUFFER_BYTES, 0, 0, nullptr, 0) == ESP_OK &&
           uart_param_config(UART_PORT, &uart_config) == ESP_OK &&
           uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK;
}

bool uart_close(uxrCustomTransport *)
{
    return uart_driver_delete(UART_PORT) == ESP_OK;
}

size_t uart_write(uxrCustomTransport *, const uint8_t *buffer, size_t length, uint8_t *error)
{
    const int written = uart_write_bytes(UART_PORT, buffer, length);
    if (written < 0) {
        *error = 1;
        return 0;
    }
    return static_cast<size_t>(written);
}

size_t uart_read(uxrCustomTransport *, uint8_t *buffer, size_t length, int timeout_ms, uint8_t *error)
{
    const int read = uart_read_bytes(UART_PORT, buffer, length, pdMS_TO_TICKS(timeout_ms));
    if (read < 0) {
        *error = 1;
        return 0;
    }
    return static_cast<size_t>(read);
}

}  // namespace

void transport_register()
{
    // Framing on: a serial stream has no message boundaries, so XRCE-DDS delimits each message.
    rmw_uros_set_custom_transport(true, nullptr, uart_open, uart_close, uart_write, uart_read);
}
