#pragma once

#include "uart_transport.h"

// Link with the sequencer (ADR 018): the micro-ROS task on core 0 and the message conversion.
// It reaches the agent through the UartTransport it owns, rebuilding the session whenever the
// agent is lost.
class Link
{
    public:
        Link();
        Link(const Link &) = delete;
        Link &operator=(const Link &) = delete;

        // Starts the micro-ROS task. Called once, at boot.
        void start();

    private:
        static void task_entry(void *link);

        // Task body. A FreeRTOS task must never return.
        [[noreturn]] void run();

        UartTransport transport_;
};
