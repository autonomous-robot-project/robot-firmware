#pragma once

// Exists only to request the test events of V-07 (ADR 049), in the same binary that operates
// (ADR 028): RESPONSE_TEST_PIN pulled to GND requests them. The pin has an internal pull-up, so
// the request is off while the pin is left open, also at boot.
class ResponseTestTrigger
{
    public:
        // Configures the pin as an input with pull-up. Called once, at boot.
        void start();

        // True while the pin is low. Read by the control task once per cycle.
        [[nodiscard]] bool requested() const;
};
