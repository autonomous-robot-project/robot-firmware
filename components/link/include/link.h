#pragma once

// Link with the sequencer: the micro-ROS task and the message conversion (ADR 018).

// Starts the micro-ROS task on its core. It waits for the agent, keeps the session and rebuilds it
// whenever the agent is lost.
void link_start();
