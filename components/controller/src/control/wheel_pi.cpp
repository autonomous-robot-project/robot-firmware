#include "control/wheel_pi.h"

#include <algorithm>
#include <cmath>

#include "control_config.h"

constexpr float BRAKE = 0.0f;

// Below v_min the speed is not observable, so it cannot be controlled (ADR 030).
static bool below_v_min(float reference)
{
    return std::abs(reference) < V_MIN;
}

float WheelPI::update(float reference, float measured)
{
    if (below_v_min(reference))
    {
        integral_ = 0.0f;
        return BRAKE;
    }

    const float error = reference - measured;
    const float integral_with_error = integral_ + KI * error * T_C_SECONDS;
    const float output = KP * error + integral_with_error;
    const float duty = std::clamp(output, -DUTY_MAX, DUTY_MAX);

    // Conditional integration (ADR 023): no accumulation while the output is saturated and the
    // error pushes it further into the saturation.
    const bool saturated = duty != output;
    const bool error_pushes_further = error * output > 0.0f;
    if (!(saturated && error_pushes_further))
    {
        integral_ = integral_with_error;
    }

    return duty;
}
