#pragma once

#include <freertos/FreeRTOS.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "messages.h"

class Controller;

// One micro-ROS session with the agent, alive while the object exists (RAII): the constructor
// creates the support, the node, the command subscription, the state publisher and the executor;
// the destructor destroys whatever was created, in reverse order.
// Not copyable nor movable: the entities keep pointers into each other.
class Session
{
    public:
        explicit Session(Controller &controller);
        ~Session();
        Session(const Session &) = delete;
        Session &operator=(const Session &) = delete;

        // False if any entity could not be created.
        [[nodiscard]] bool ok() const { return ready_; }

        // Waits for commands until the deadline, handing each one to the controller as it arrives.
        void receive_commands_until(TickType_t deadline);

        // Publishes the latest state of the controller, if there is one yet.
        void publish_state();

    private:
        // Each one creates an entity, logs a failure and returns whether it succeeded.
        bool create_support();
        bool create_node();
        bool create_command_subscription();
        bool create_state_publisher();
        bool create_executor();

        void destroy_executor();
        void destroy_state_publisher();
        void destroy_command_subscription();
        void destroy_node();
        void destroy_support();

        // C callback of the subscription; the session comes back through the context.
        static void on_command(const void *message, void *session);

        Controller &controller_;

        rcl_allocator_t allocator_ = rcl_get_default_allocator();
        rclc_support_t support_{};
        rcl_node_t node_ = rcl_get_zero_initialized_node();
        rcl_subscription_t command_subscription_ = rcl_get_zero_initialized_subscription();
        rcl_publisher_t state_publisher_ = rcl_get_zero_initialized_publisher();
        rclc_executor_t executor_ = rclc_executor_get_zero_initialized_executor();

        CommandMessage command_message_{};
        StateMessage state_message_{};

        bool support_ok_ = false;
        bool node_ok_ = false;
        bool subscription_ok_ = false;
        bool publisher_ok_ = false;
        bool executor_ok_ = false;
        bool ready_ = false;  // every entity created
};
