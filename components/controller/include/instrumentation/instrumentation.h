#pragma once

#include "control/wheel_pair.h"
#include "instrumentation/deadline_meter.h"
#include "instrumentation/expiry_meter.h"

// Measurements of the controller for the verifications (ADR 028), one part per meter.
// The meters accumulate since boot; the wheel speeds are those of the latest cycle.
struct Instrumentation
{
    public:
        ExpiryMeter::Measurements expiry;       // V-06
        DeadlineMeter::Measurements deadlines;  // V-05
        WheelPair wheel_speeds;                 // m/s, positive forward; zero below v_min (V-02)
};
