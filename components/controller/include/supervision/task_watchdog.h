#pragma once

// Task watchdog over the control task (ADR 029). If a cycle does not feed it within
// WATCHDOG_TIMEOUT, the chip panics and restarts, and the drivers fall to the safe state
// (ADR 008). Only the control task is watched: the other tasks already degrade safely.
class TaskWatchdog
{
    public:
        // Configures the watchdog. Called once, at boot, from core 0, so that its interrupt
        // stays off the core of the control task.
        void start();

        // Subscribes the calling task. Called once, by the control task, before its first cycle.
        void watch_calling_task();

        // Called by the control task at the end of every cycle.
        void feed();

        // True if the last restart of the chip was caused by this watchdog. Read at boot.
        [[nodiscard]] static bool caused_last_restart();
};
