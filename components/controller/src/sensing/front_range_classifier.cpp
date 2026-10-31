#include "sensing/front_range_classifier.h"

#include "control_config.h"

// Device status of the VL53L0X seen in the trial without power (robot-docs nota 0015); the names
// are those of vl53l0x_device.h of the ST API.
static constexpr uint8_t RANGE_COMPLETE = 11;
static constexpr uint8_t RANGE_PHASE_CHECK = 6;
static constexpr uint8_t PHASE_CONSISTENCY = 9;

static constexpr float MM_PER_M = 1000.0f;

// The condition a ranging gives, or empty if it is invalid (robot-docs nota 0015, D6).
static std::optional<FrontCondition> condition_of(const FrontRange &range)
{
    const bool no_target_status = range.device_status == RANGE_PHASE_CHECK ||
                                  range.device_status == PHASE_CONSISTENCY;
    if (no_target_status)
    {
        return FrontCondition::CLEAR;
    }

    const bool complete_with_spads = range.device_status == RANGE_COMPLETE &&
                                     range.effective_spads_q8_8 != 0;
    if (!complete_with_spads)
    {
        return std::nullopt;
    }

    // The crosstalk of the cover reads as a complete ranging with a weak signal: no target.
    if (range.signal_rate_per_spad() < S_MIN_VLX)
    {
        return FrontCondition::CLEAR;
    }

    const bool below_d_stop = static_cast<float>(range.distance_mm) / MM_PER_M < D_STOP;
    return below_d_stop ? FrontCondition::OBSTACLE : FrontCondition::CLEAR;
}

FrontCondition FrontRangeClassifier::classify(
    const std::optional<FrontRange> &latest, std::chrono::microseconds now
)
{
    if (latest)
    {
        const std::optional<FrontCondition> condition = condition_of(*latest);
        if (condition)
        {
            last_usable_ = UsableRanging{
                .condition = *condition, .measured_at = latest->measured_at
            };
        }
    }

    const bool too_old = last_usable_ && now - last_usable_->measured_at > T_S;
    if (!last_usable_ || too_old)
    {
        return FrontCondition::FAILURE;
    }

    return last_usable_->condition;
}
