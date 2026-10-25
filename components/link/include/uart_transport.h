#pragma once

#include <cstddef>
#include <cstdint>

struct uxrCustomTransport;

// Byte transport of micro-ROS over a UART (ADRs 012 and 043): how the bytes reach the agent.
// It knows nothing about messages; micro-ROS calls it from the link task.
class UartTransport
{
    public:
        struct Config
        {
            public:  // explicit, so the members indent as in a class
                int port;
                int tx_pin;
                int rx_pin;
                int baud;
                int rx_buffer_bytes;
        };

        explicit UartTransport(const Config &config) : config_(config) {}
        UartTransport(const UartTransport &) = delete;
        UartTransport &operator=(const UartTransport &) = delete;

        // Makes this UART the custom transport of micro-ROS. The object must outlive every
        // session.
        void register_with_micro_ros();

    private:
        // C callbacks called by micro-ROS; each one recovers the object from transport->args.
        static bool open(uxrCustomTransport *transport);
        static bool close(uxrCustomTransport *transport);
        static size_t write(
            uxrCustomTransport *transport, const uint8_t *buffer, size_t length, uint8_t *error
        );
        static size_t read(
            uxrCustomTransport *transport,
            uint8_t *buffer,
            size_t length,
            int timeout_ms,
            uint8_t *error
        );

        static const Config &config_of(uxrCustomTransport *transport);

        const Config config_;
};
