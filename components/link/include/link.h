#pragma once

#include "uart_transport.h"

class Agent;
class Controller;
class Session;

// Link with the sequencer (ADR 018): the micro-ROS task on core 0 and the message conversion.
// It hands the received commands to the controller and publishes its state, rebuilding the
// session whenever the agent is lost. It depends on the controller, never the opposite.
class Link
{
    public:
        explicit Link(Controller &controller);
        Link(const Link &) = delete;
        Link &operator=(const Link &) = delete;

        // Starts the micro-ROS task. Called once, at boot.
        void start();

    private:
        static void task_entry(void *link);

        // Task body. A FreeRTOS task must never return.
        [[noreturn]] void run();

        // Exchanges commands and state every STATE_PERIOD until the agent is lost.
        void serve(Session &session, const Agent &agent);

        Controller &controller_;
        UartTransport transport_;
};
