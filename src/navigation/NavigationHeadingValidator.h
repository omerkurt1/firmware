#pragma once

#include "navigation/NavigationTypes.h"

enum class NavigationHeadingReference : uint8_t { UNKNOWN, TRUE_NORTH, MAGNETIC_NORTH };

struct NavigationHeadingSample {
    float degrees = 0;
    NavigationHeadingSource source = NavigationHeadingSource::NONE;
    NavigationHeadingReference reference = NavigationHeadingReference::UNKNOWN;
    // Measurement time in this boot's monotonic clock, never packet receipt or display time.
    uint64_t measuredAtUptimeMs = 0;
    bool timeKnown = false;
    bool measurementValid = false;
    bool calibrationValid = false;
    bool deviceOrientationValid = false;
    bool fixValid = false;
    float speedMetersPerSecond = 0;
    bool speedValid = false;
    uint64_t speedMeasuredAtUptimeMs = 0;
    bool speedTimeKnown = false;
};

struct NavigationHeadingPolicy {
    uint32_t maxAgeMillis = 2000;
    uint32_t maxCourseSpeedSkewMillis = 500;
    float minMovementSpeedMetersPerSecond = 1.0f;
};

struct NavigationHeadingEvaluation {
    NavigationHeading heading;
    NavigationHeadingState state = NavigationHeadingState::NO_SAMPLE;
    uint64_t ageMillis = 0;
    bool ageKnown = false;
};

class NavigationHeadingValidator
{
  public:
    static bool isPolicyValid(const NavigationHeadingPolicy &policy);
    static NavigationHeadingEvaluation evaluate(const NavigationHeadingSample &sample, uint64_t nowUptimeMs,
                                                const NavigationHeadingPolicy &policy = {});
};
