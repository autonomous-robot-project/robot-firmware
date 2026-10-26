#pragma once

// PI on the speed of one wheel, with its output as the duty of the motor. The gains come from the
// motor model and the settling time (config.h). Below v_min the speed is not observable, so the
// duty is the brake and the integrator is reset (ADR 030). The integrator does not accumulate
// while the output is saturated and the error pushes it further (ADR 023): it then stays within
// DUTY_MAX plus one step.
class WheelPI
{
    public:
        // Duty for this cycle, saturated at DUTY_MAX.
        [[nodiscard]] float update(float reference, float measured);

    private:
        float integral_ = 0.0f;  // the integral term, in duty
};
