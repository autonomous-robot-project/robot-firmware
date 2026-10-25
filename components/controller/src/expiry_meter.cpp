#include "expiry_meter.h"

void ExpiryMeter::record(std::chrono::microseconds arrival, std::chrono::microseconds cycle)
{
    const bool already_counted = last_expired_arrival_ == arrival;
    if (already_counted)
    {
        return;
    }
    last_expired_arrival_ = arrival;
    measurements_.expiries++;
    measurements_.expiry_delay_us = static_cast<uint32_t>((cycle - arrival).count());
}
