// Unit test of FrontRangeClassifier on the host, with the rangings of the trial without power
// (robot-docs nota 0015): the screen of V-03 and the one at ~1 cm are obstacles, the crosstalk of
// the cover and the no-target status are clear, an invalid ranging keeps the last condition for
// T_S, and no usable ranging for more than T_S, or none since boot, is a failure (ADR 021).

#include <chrono>
#include <cstdio>

#include "check.h"
#include "control_config.h"
#include "sensing/front_range_classifier.h"

using std::chrono::microseconds;

// Device status of the trial (robot-docs nota 0015); 4 was not seen, so it is invalid.
constexpr uint8_t RANGE_COMPLETE = 11;
constexpr uint8_t RANGE_PHASE_CHECK = 6;
constexpr uint8_t PHASE_CONSISTENCY = 9;
constexpr uint8_t UNSEEN_STATUS = 4;

constexpr uint16_t OUT_OF_RANGE_MM = 8190;  // distance the sensor reports with no target

// Medians of the trial, in the fixed-point fields of the sensor (signal Q9.7, SPADs Q8.8).
constexpr FrontRange SCREEN_AT_100_MM = {
    .distance_mm = 97,
    .device_status = RANGE_COMPLETE,
    .signal_rate_q9_7 = 2968,      // 23.2 MCPS
    .effective_spads_q8_8 = 5124,  // 20.0
};
constexpr FrontRange SCREEN_AT_1_CM = {
    .distance_mm = 15,
    .device_status = RANGE_COMPLETE,
    .signal_rate_q9_7 = 4022,     // 31.4 MCPS
    .effective_spads_q8_8 = 468,  // 1.8
};
constexpr FrontRange COVER_CROSSTALK = {
    .distance_mm = 0,
    .device_status = RANGE_COMPLETE,
    .signal_rate_q9_7 = 47,         // 0.37 MCPS
    .effective_spads_q8_8 = 49156,  // 192.0
};

static FrontRange at(FrontRange range, microseconds measured_at)
{
    range.measured_at = measured_at;
    return range;
}

static FrontRange with_status(uint8_t status, microseconds measured_at)
{
    return {.distance_mm = OUT_OF_RANGE_MM, .device_status = status, .measured_at = measured_at};
}

static FrontRange screen_at_distance(uint16_t distance_mm)
{
    FrontRange range = SCREEN_AT_100_MM;
    range.distance_mm = distance_mm;
    return range;
}

static FrontCondition classify_once(const FrontRange &range)
{
    FrontRangeClassifier classifier;
    return classifier.classify(range, range.measured_at);
}

static void check_conditions_of_the_trial()
{
    check(
        classify_once(SCREEN_AT_100_MM) == FrontCondition::OBSTACLE, "screen at 100 mm: obstacle"
    );
    check(classify_once(SCREEN_AT_1_CM) == FrontCondition::OBSTACLE, "screen at ~1 cm: obstacle");
    check(
        classify_once(COVER_CROSSTALK) == FrontCondition::CLEAR, "crosstalk of the cover: clear"
    );
    check(
        classify_once(with_status(RANGE_PHASE_CHECK, {})) == FrontCondition::CLEAR,
        "status 6: clear"
    );
    check(
        classify_once(with_status(PHASE_CONSISTENCY, {})) == FrontCondition::CLEAR,
        "status 9: clear"
    );
}

static void check_d_stop_boundary()
{
    constexpr auto d_stop_mm = static_cast<uint16_t>(D_STOP * 1000.0f);
    check(
        classify_once(screen_at_distance(d_stop_mm - 1)) == FrontCondition::OBSTACLE,
        "1 mm below d_stop: obstacle"
    );
    check(
        classify_once(screen_at_distance(d_stop_mm)) == FrontCondition::CLEAR, "at d_stop: clear"
    );
}

static void check_invalid_rangings()
{
    FrontRangeClassifier boot;
    check(
        boot.classify(std::nullopt, {}) == FrontCondition::FAILURE, "no ranging at boot: failure"
    );

    FrontRangeClassifier invalid_only;
    check(
        invalid_only.classify(with_status(UNSEEN_STATUS, {}), {}) == FrontCondition::FAILURE,
        "only an invalid ranging since boot: failure"
    );

    FrontRange no_spads = SCREEN_AT_100_MM;
    no_spads.effective_spads_q8_8 = 0;
    check(classify_once(no_spads) == FrontCondition::FAILURE, "complete without SPADs: invalid");
}

static void check_age()
{
    const microseconds t0{1'000'000};
    const microseconds t_s = T_S;
    const microseconds one_us{1};

    FrontRangeClassifier classifier;
    check(
        classifier.classify(at(SCREEN_AT_100_MM, t0), t0) == FrontCondition::OBSTACLE,
        "screen at t0: obstacle"
    );
    check(
        classifier.classify(with_status(UNSEEN_STATUS, t0 + t_s / 2), t0 + t_s) ==
            FrontCondition::OBSTACLE,
        "invalid ranging within T_S: last condition kept"
    );
    check(
        classifier.classify(with_status(UNSEEN_STATUS, t0 + t_s), t0 + t_s + one_us) ==
            FrontCondition::FAILURE,
        "invalid rangings for more than T_S: failure"
    );
    check(
        classifier.classify(at(COVER_CROSSTALK, t0 + 2 * t_s), t0 + 2 * t_s) ==
            FrontCondition::CLEAR,
        "usable ranging after the failure: recovered"
    );
    check(
        classifier.classify(at(COVER_CROSSTALK, t0 + 2 * t_s), t0 + 3 * t_s + one_us) ==
            FrontCondition::FAILURE,
        "same ranging for more than T_S (sensor task stopped): failure"
    );
}

int main()
{
    check_conditions_of_the_trial();
    check_d_stop_boundary();
    check_invalid_rangings();
    check_age();
    return verdict();
}
