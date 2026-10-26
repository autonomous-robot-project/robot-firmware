#include "supervision/wheel_tracking_monitor.h"

#include <cmath>

#include "control_config.h"

bool WheelTrackingMonitor::update(float reference, float measured, std::chrono::microseconds now)
{
    const bool observable = std::abs(reference) >= V_MIN;
    const bool error_above_limit = std::abs(reference - measured) > E_F * std::abs(reference);
    if (!(observable && error_above_limit))
    {
        error_since_.reset();
        return false;
    }

    if (!error_since_)
    {
        error_since_ = now;
    }

    const float persisted = std::chrono::duration<float>(now - *error_since_).count();
    return persisted >= T_F;
}
