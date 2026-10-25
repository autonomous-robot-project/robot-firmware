#pragma once

#include <driver/ledc.h>

#include "actuation/driver_input.h"

// One DRV8871 in slow decay (ADR 023): one input stays high and the other is pulsed low. Low
// drives the motor and both high brake it. Both low leave the wheel free, the state from the
// reset to the first control cycle (ADR 008).
class MotorDriver
{
    public:
        // Where the driver is wired: its own LEDC timer and its two inputs.
        struct Wiring
        {
            public:
                ledc_timer_t timer;
                DriverInput::Wiring in1;
                DriverInput::Wiring in2;
        };

        explicit MotorDriver(const Wiring &wiring);

        // Configures the PWM with the wheel free. Called once, at boot, before the control task.
        void start();

        // Drive duty from -1 to 1: positive turns the motor forward, negative backward, and zero
        // brakes it. Protection of the motor, whatever the caller: the duty never exceeds
        // DUTY_MAX, so a stalled wheel gets at most the rated voltage (ADR 023). The PI saturates
        // before, so this limit does not act in normal operation.
        void apply(float duty);

    private:
        ledc_timer_t timer_;
        DriverInput in1_;
        DriverInput in2_;
};
