#include "navigation/NavigationHeadingValidator.h"

#include <cmath>

bool NavigationHeadingValidator::isPolicyValid(const NavigationHeadingPolicy &policy)
{
    return policy.maxAgeMillis > 0 && policy.maxCourseSpeedSkewMillis <= policy.maxAgeMillis &&
           std::isfinite(policy.minMovementSpeedMetersPerSecond) && policy.minMovementSpeedMetersPerSecond > 0;
}

NavigationHeadingEvaluation NavigationHeadingValidator::evaluate(const NavigationHeadingSample &sample, uint64_t nowUptimeMs,
                                                                 const NavigationHeadingPolicy &policy)
{
    NavigationHeadingEvaluation result;
    auto reject = [&](NavigationHeadingState state) {
        result.state = state;
        return result;
    };
    if (!isPolicyValid(policy))
        return reject(NavigationHeadingState::INVALID_POLICY);
    if (sample.source == NavigationHeadingSource::NONE)
        return result;
    if (sample.source != NavigationHeadingSource::MAGNETOMETER && sample.source != NavigationHeadingSource::MOVEMENT_COURSE)
        return reject(NavigationHeadingState::INVALID_SOURCE);
    if (!sample.measurementValid || !std::isfinite(sample.degrees) || sample.degrees < 0 || sample.degrees >= 360)
        return reject(NavigationHeadingState::INVALID_SAMPLE);
    if (!sample.timeKnown)
        return reject(NavigationHeadingState::TIME_UNKNOWN);
    if (sample.measuredAtUptimeMs > nowUptimeMs)
        return reject(NavigationHeadingState::FUTURE_SAMPLE);
    result.ageKnown = true;
    result.ageMillis = nowUptimeMs - sample.measuredAtUptimeMs;
    if (result.ageMillis >= policy.maxAgeMillis)
        return reject(NavigationHeadingState::STALE);
    if (sample.reference != NavigationHeadingReference::TRUE_NORTH)
        return reject(NavigationHeadingState::REFERENCE_UNVERIFIED);

    if (sample.source == NavigationHeadingSource::MAGNETOMETER) {
        if (!sample.calibrationValid)
            return reject(NavigationHeadingState::UNCALIBRATED);
        if (!sample.deviceOrientationValid)
            return reject(NavigationHeadingState::ORIENTATION_UNVERIFIED);
    } else {
        if (!sample.fixValid)
            return reject(NavigationHeadingState::INVALID_FIX);
        if (!sample.speedValid || !std::isfinite(sample.speedMetersPerSecond) || sample.speedMetersPerSecond < 0)
            return reject(NavigationHeadingState::INVALID_SPEED);
        if (!sample.speedTimeKnown)
            return reject(NavigationHeadingState::SPEED_TIME_UNKNOWN);
        if (sample.speedMeasuredAtUptimeMs > nowUptimeMs)
            return reject(NavigationHeadingState::FUTURE_SAMPLE);
        if (nowUptimeMs - sample.speedMeasuredAtUptimeMs >= policy.maxAgeMillis)
            return reject(NavigationHeadingState::SPEED_STALE);
        const uint64_t skew = sample.measuredAtUptimeMs >= sample.speedMeasuredAtUptimeMs
                                  ? sample.measuredAtUptimeMs - sample.speedMeasuredAtUptimeMs
                                  : sample.speedMeasuredAtUptimeMs - sample.measuredAtUptimeMs;
        if (skew > policy.maxCourseSpeedSkewMillis)
            return reject(NavigationHeadingState::SPEED_UNSYNCHRONIZED);
        if (sample.speedMetersPerSecond < policy.minMovementSpeedMetersPerSecond)
            return reject(NavigationHeadingState::LOW_SPEED);
    }

    result.heading = {.degrees = sample.degrees, .source = sample.source, .valid = true};
    result.state = NavigationHeadingState::READY;
    return result;
}
