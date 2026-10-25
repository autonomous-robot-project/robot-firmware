#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>

// One micro-ROS session with the agent, alive while the object exists (RAII): the constructor
// creates the support and the node, the destructor destroys them.
// Not copyable nor movable: the node keeps pointers into the support.
class Session
{
    public:
        Session();
        ~Session();
        Session(const Session &) = delete;
        Session &operator=(const Session &) = delete;

        // False if the support or the node could not be created.
        [[nodiscard]] bool ok() const { return node_ok_; }

    private:
        rcl_allocator_t allocator_ = rcl_get_default_allocator();
        rclc_support_t support_{};
        rcl_node_t node_ = rcl_get_zero_initialized_node();
        bool support_ok_ = false;
        bool node_ok_ = false;
};
