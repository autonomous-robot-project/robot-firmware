#pragma once

// Liveness of the micro-ROS agent on the RPi, checked by ping.
class Agent
{
    public:
        [[nodiscard]] bool is_alive() const;

        // Blocks the calling task until the agent answers.
        void wait_until_alive() const;
};
