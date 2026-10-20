#include "navigation/NavigationCalculator.h"

#include "gps/GeoCoord.h"
#include <cmath>

namespace
{
constexpr double COORDINATE_SCALE = 1e-7;
constexpr int32_t MAX_LATITUDE_I = 900000000;
constexpr int32_t MAX_LONGITUDE_I = 1800000000;
constexpr float RADIANS_TO_DEGREES = 57.29577951308232f;
} // namespace

bool NavigationCalculator::isCoordinateValid(const NavigationCoordinate &coordinate)
{
    const bool inRange = coordinate.latitudeI >= -MAX_LATITUDE_I && coordinate.latitudeI <= MAX_LATITUDE_I &&
                         coordinate.longitudeI >= -MAX_LONGITUDE_I && coordinate.longitudeI <= MAX_LONGITUDE_I;
    return inRange && (coordinate.latitudeI != 0 || coordinate.longitudeI != 0);
}

float NavigationCalculator::normalizeDegrees360(float degrees)
{
    if (!std::isfinite(degrees))
        return 0.0f;

    float normalized = fmodf(degrees, 360.0f);
    if (normalized < 0.0f)
        normalized += 360.0f;
    return normalized;
}

float NavigationCalculator::normalizeDeltaDegrees180(float degrees)
{
    float normalized = normalizeDegrees360(degrees);
    if (normalized >= 180.0f)
        normalized -= 360.0f;
    return normalized;
}

NavigationGeometry NavigationCalculator::calculate(const NavigationCoordinate &origin, const NavigationCoordinate &target,
                                                   const NavigationHeading &heading)
{
    NavigationGeometry result;
    if (!isCoordinateValid(origin) || !isCoordinateValid(target))
        return result;

    const double originLatitude = origin.latitudeI * COORDINATE_SCALE;
    const double originLongitude = origin.longitudeI * COORDINATE_SCALE;
    const double targetLatitude = target.latitudeI * COORDINATE_SCALE;
    const double targetLongitude = target.longitudeI * COORDINATE_SCALE;

    result.distanceMeters = GeoCoord::latLongToMeter(originLatitude, originLongitude, targetLatitude, targetLongitude);
    result.distanceValid = std::isfinite(result.distanceMeters) && result.distanceMeters >= 0.0f;
    if (!result.distanceValid || result.distanceMeters == 0.0f)
        return result;

    const float bearingRadians = GeoCoord::bearing(originLatitude, originLongitude, targetLatitude, targetLongitude);
    if (!std::isfinite(bearingRadians))
        return result;

    result.absoluteBearingDegrees = normalizeDegrees360(bearingRadians * RADIANS_TO_DEGREES);
    result.bearingValid = true;

    if (heading.valid && heading.source != NavigationHeadingSource::NONE && std::isfinite(heading.degrees)) {
        result.relativeBearingDegrees = normalizeDeltaDegrees180(result.absoluteBearingDegrees - heading.degrees);
        result.relativeBearingValid = true;
    }

    return result;
}
