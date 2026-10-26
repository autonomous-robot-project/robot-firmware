#pragma once

// Exists only to test the task watchdog in V-02 (ADR 029), in the same binary that operates
// (ADR 028): WATCHDOG_TEST_PIN pulled to GND hangs the control task in an endless loop, which
// only the watchdog can end. The pin has an internal pull-up, so the trigger is inert while the
// pin is left open.
class WatchdogTestTrigger
{
    public:
        // Configures the pin as an input with pull-up. Called once, at boot.
        void start();

        // Called by the control task once per cycle. Never returns while the pin is low.
        void hang_if_requested() const;
};
