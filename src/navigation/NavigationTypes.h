#pragma once

#include <cstdint>

enum class NavigationPositionState : uint8_t {
    READY,
    NO_LOCAL_POSITION,
    NO_TARGET_POSITION,
    LOCAL_POSITION_STALE,
    TARGET_POSITION_STALE,
    POSITION_AGE_UNKNOWN,
    NO_TARGET_SELECTED,
};

enum class NavigationHeadingSource : uint8_t {
    MAGNETOMETER,
    MOVEMENT_COURSE,
    NONE,
};

enum class NavigationHeadingState : uint8_t {
    NO_SAMPLE,
    READY,
    INVALID_POLICY,
    INVALID_SOURCE,
    INVALID_SAMPLE,
    TIME_UNKNOWN,
    FUTURE_SAMPLE,
    STALE,
    REFERENCE_UNVERIFIED,
    UNCALIBRATED,
    ORIENTATION_UNVERIFIED,
    INVALID_FIX,
    INVALID_SPEED,
    SPEED_TIME_UNKNOWN,
    SPEED_STALE,
    SPEED_UNSYNCHRONIZED,
    LOW_SPEED,
};

struct NavigationCoordinate {
    int32_t latitudeI = 0;
    int32_t longitudeI = 0;
};

struct NavigationHeading {
    float degrees = 0.0f;
    NavigationHeadingSource source = NavigationHeadingSource::NONE;
    bool valid = false;
};

struct NavigationGeometry {
    float distanceMeters = 0.0f;
    float absoluteBearingDegrees = 0.0f;
    float relativeBearingDegrees = 0.0f;
    bool distanceValid = false;
    bool bearingValid = false;
    bool relativeBearingValid = false;
};

struct NavigationSnapshot {
    uint32_t targetNode = 0;
    NavigationPositionState positionState = NavigationPositionState::POSITION_AGE_UNKNOWN;
    NavigationHeadingSource headingSource = NavigationHeadingSource::NONE;
    NavigationHeadingState headingState = NavigationHeadingState::NO_SAMPLE;
    uint64_t headingAgeMillis = 0;
    bool headingAgeKnown = false;
    float distanceMeters = 0.0f;
    float absoluteBearingDegrees = 0.0f;
    float relativeBearingDegrees = 0.0f;
    uint32_t localPositionAgeSeconds = 0;
    uint32_t targetPositionAgeSeconds = 0;
    uint8_t targetPrecisionBits = 0;
    bool distanceValid = false;
    bool bearingValid = false;
    bool arrowValid = false;
    bool localPositionAgeKnown = false;
    bool targetPositionAgeKnown = false;
    bool localPositionWarning = false;
    bool targetPositionWarning = false;
};
