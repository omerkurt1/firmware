#pragma once

#include <cstdint>

enum class NavigationPositionState : uint8_t {
    READY,
    NO_LOCAL_POSITION,
    NO_TARGET_POSITION,
    LOCAL_POSITION_STALE,
    TARGET_POSITION_STALE,
    POSITION_AGE_UNKNOWN,
};

enum class NavigationHeadingSource : uint8_t {
    MAGNETOMETER,
    MOVEMENT_COURSE,
    NONE,
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
    float distanceMeters = 0.0f;
    float absoluteBearingDegrees = 0.0f;
    float relativeBearingDegrees = 0.0f;
    uint32_t localPositionAgeSeconds = 0;
    uint32_t targetPositionAgeSeconds = 0;
    uint8_t targetPrecisionBits = 0;
    bool distanceValid = false;
    bool bearingValid = false;
    bool arrowValid = false;
};
