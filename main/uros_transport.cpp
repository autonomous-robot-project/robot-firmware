#include "uros_transport.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "robot_config.h"

static bool serial_open(uxrCustomTransport *)
{
    const uart_config_t cfg = {
        .baud_rate = UROS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    return uart_param_config(UROS_UART_PORT, &cfg) == ESP_OK &&
           uart_set_pin(UROS_UART_PORT, UROS_UART_TX, UROS_UART_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
           uart_driver_install(UROS_UART_PORT, UROS_UART_BUF_BYTES, 0, 0, nullptr, 0) == ESP_OK;
}

static bool serial_close(uxrCustomTransport *)
{
    return uart_driver_delete(UROS_UART_PORT) == ESP_OK;
}

static size_t serial_write(uxrCustomTransport *, const uint8_t *buf, size_t len, uint8_t *err)
{
    const int n = uart_write_bytes(UROS_UART_PORT, buf, len);
    if (n < 0) {
        *err = 1;
        return 0;
    }
    return static_cast<size_t>(n);
}

static size_t serial_read(uxrCustomTransport *, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    const int n = uart_read_bytes(UROS_UART_PORT, buf, len, pdMS_TO_TICKS(timeout));
    if (n < 0) {
        *err = 1;
        return 0;
    }
    return static_cast<size_t>(n);
}

void uros_transport_register()
{
    rmw_uros_set_custom_transport(true, nullptr, serial_open, serial_close, serial_write, serial_read);
}
