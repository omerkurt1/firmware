// NavigationHeadingValidator in src/navigation/NavigationHeadingValidator.cpp admits only fresh,
// true-north measurements with source-specific quality metadata. These tests guard stale compass
// reuse, retained GNSS speed/course mismatches, unknown timestamps and 32-bit clock wrap regressions.
#include "MeshTypes.h"
#include "TestUtil.h"
#include "navigation/NavigationHeadingValidator.h"
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <unity.h>

namespace
{
constexpr uint64_t NOW = 10000;

NavigationHeadingSample compass()
{
    NavigationHeadingSample sample;
    sample.degrees = 90;
    sample.source = NavigationHeadingSource::MAGNETOMETER;
    sample.reference = NavigationHeadingReference::TRUE_NORTH;
    sample.measuredAtUptimeMs = NOW;
    sample.timeKnown = sample.measurementValid = sample.calibrationValid = sample.deviceOrientationValid = true;
    return sample;
}

NavigationHeadingSample course()
{
    auto sample = compass();
    sample.source = NavigationHeadingSource::MOVEMENT_COURSE;
    sample.calibrationValid = sample.deviceOrientationValid = false;
    sample.fixValid = sample.speedValid = sample.speedTimeKnown = true;
    sample.speedMetersPerSecond = 2;
    sample.speedMeasuredAtUptimeMs = NOW;
    return sample;
}

void assertRejected(const NavigationHeadingSample &sample, NavigationHeadingState state, uint64_t now = NOW,
                    const NavigationHeadingPolicy &policy = {})
{
    const auto result = NavigationHeadingValidator::evaluate(sample, now, policy);
    TEST_ASSERT_EQUAL(state, result.state);
    TEST_ASSERT_FALSE(result.heading.valid);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::NONE, result.heading.source);
    TEST_ASSERT_EQUAL_FLOAT(0, result.heading.degrees);
}
} // namespace

void setUp() {}
void tearDown() {}

static void test_sample_defaultIsUnavailable()
{
    assertRejected({}, NavigationHeadingState::NO_SAMPLE);
    TEST_ASSERT_FALSE(NavigationHeadingValidator::evaluate({}, NOW).ageKnown);
}

static void test_compass_validSamplePreservesAngleAndSource()
{
    const auto result = NavigationHeadingValidator::evaluate(compass(), NOW);
    TEST_ASSERT_EQUAL(NavigationHeadingState::READY, result.state);
    TEST_ASSERT_TRUE(result.heading.valid);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::MAGNETOMETER, result.heading.source);
    TEST_ASSERT_EQUAL_FLOAT(90, result.heading.degrees);
    TEST_ASSERT_TRUE(result.ageKnown);
    TEST_ASSERT_EQUAL_UINT64(0, result.ageMillis);
}

static void test_course_validSampleDoesNotRequireCompassCalibration()
{
    const auto result = NavigationHeadingValidator::evaluate(course(), NOW);
    TEST_ASSERT_EQUAL(NavigationHeadingState::READY, result.state);
    TEST_ASSERT_TRUE(result.heading.valid);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::MOVEMENT_COURSE, result.heading.source);
}

static void test_sample_invalidSourceAndValidityAreRejected()
{
    auto sample = compass();
    sample.source = static_cast<NavigationHeadingSource>(255);
    assertRejected(sample, NavigationHeadingState::INVALID_SOURCE);
    sample = compass();
    sample.measurementValid = false;
    assertRejected(sample, NavigationHeadingState::INVALID_SAMPLE);
}

static void test_angle_nonFiniteAndOutOfRangeAreRejected()
{
    auto sample = compass();
    for (float angle : {NAN, INFINITY, -INFINITY, -1.0f, 360.0f, 720.0f}) {
        sample.degrees = angle;
        assertRejected(sample, NavigationHeadingState::INVALID_SAMPLE);
    }
    for (float angle : {0.0f, 359.99f}) {
        sample.degrees = angle;
        TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW).heading.valid);
    }
}

static void test_time_unknownIsDifferentFromBootZero()
{
    auto sample = compass();
    sample.measuredAtUptimeMs = 0;
    sample.timeKnown = false;
    assertRejected(sample, NavigationHeadingState::TIME_UNKNOWN, 0);
    TEST_ASSERT_FALSE(NavigationHeadingValidator::evaluate(sample, 0).ageKnown);
    sample.timeKnown = true;
    const auto result = NavigationHeadingValidator::evaluate(sample, 0);
    TEST_ASSERT_TRUE(result.heading.valid);
    TEST_ASSERT_TRUE(result.ageKnown);
    TEST_ASSERT_EQUAL_UINT64(0, result.ageMillis);
}

static void test_time_futureMeasurementIsRejected()
{
    auto sample = compass();
    sample.measuredAtUptimeMs = NOW + 1;
    assertRejected(sample, NavigationHeadingState::FUTURE_SAMPLE);
    TEST_ASSERT_FALSE(NavigationHeadingValidator::evaluate(sample, NOW).ageKnown);
}

static void test_time_expiryIsInclusiveAndPreservesKnownAge()
{
    const auto sample = compass();
    TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW + 1999).heading.valid);
    assertRejected(sample, NavigationHeadingState::STALE, NOW + 2000);
    const auto result = NavigationHeadingValidator::evaluate(sample, NOW + 2000);
    TEST_ASSERT_TRUE(result.ageKnown);
    TEST_ASSERT_EQUAL_UINT64(2000, result.ageMillis);
}

static void test_time_extendedClockDoesNotTruncateAcrossWrap()
{
    auto sample = compass();
    sample.measuredAtUptimeMs = UINT32_MAX - 500;
    const uint64_t now = static_cast<uint64_t>(UINT32_MAX) + 500;
    const auto result = NavigationHeadingValidator::evaluate(sample, now);
    TEST_ASSERT_TRUE(result.heading.valid);
    TEST_ASSERT_EQUAL_UINT64(1000, result.ageMillis);
    assertRejected(sample, NavigationHeadingState::STALE, now + 1000);
    const auto ancient = NavigationHeadingValidator::evaluate(sample, now + (uint64_t{1} << 32));
    TEST_ASSERT_EQUAL(NavigationHeadingState::STALE, ancient.state);
    TEST_ASSERT_EQUAL_UINT64((uint64_t{1} << 32) + 1000, ancient.ageMillis);
}

static void test_reference_requiresExplicitTrueNorth()
{
    for (auto sample : {compass(), course()}) {
        for (auto reference : {NavigationHeadingReference::UNKNOWN, NavigationHeadingReference::MAGNETIC_NORTH,
                               static_cast<NavigationHeadingReference>(255)}) {
            sample.reference = reference;
            assertRejected(sample, NavigationHeadingState::REFERENCE_UNVERIFIED);
        }
    }
}

static void test_compass_requiresCalibrationAndOrientation()
{
    auto sample = compass();
    sample.calibrationValid = false;
    assertRejected(sample, NavigationHeadingState::UNCALIBRATED);
    sample = compass();
    sample.deviceOrientationValid = false;
    assertRejected(sample, NavigationHeadingState::ORIENTATION_UNVERIFIED);
}

static void test_course_requiresValidFixAndSpeed()
{
    auto sample = course();
    sample.fixValid = false;
    assertRejected(sample, NavigationHeadingState::INVALID_FIX);
    sample = course();
    sample.speedValid = false;
    assertRejected(sample, NavigationHeadingState::INVALID_SPEED);
    sample.speedValid = true;
    for (float speed : {NAN, INFINITY, -INFINITY, -1.0f}) {
        sample.speedMetersPerSecond = speed;
        assertRejected(sample, NavigationHeadingState::INVALID_SPEED);
    }
}

static void test_course_speedTimestampMustBeKnownCurrentAndNotFuture()
{
    auto sample = course();
    sample.speedTimeKnown = false;
    assertRejected(sample, NavigationHeadingState::SPEED_TIME_UNKNOWN);
    sample.speedTimeKnown = true;
    sample.speedMeasuredAtUptimeMs = NOW + 1;
    assertRejected(sample, NavigationHeadingState::FUTURE_SAMPLE);
    sample.speedMeasuredAtUptimeMs = NOW - 2000;
    assertRejected(sample, NavigationHeadingState::SPEED_STALE);
}

static void test_course_speedAndCourseSkewIsBoundedInBothDirections()
{
    auto sample = course();
    sample.speedMeasuredAtUptimeMs = NOW - 500;
    TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW).heading.valid);
    sample.speedMeasuredAtUptimeMs--;
    assertRejected(sample, NavigationHeadingState::SPEED_UNSYNCHRONIZED);
    sample = course();
    sample.measuredAtUptimeMs = NOW - 500;
    TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW).heading.valid);
    sample.measuredAtUptimeMs--;
    assertRejected(sample, NavigationHeadingState::SPEED_UNSYNCHRONIZED);
}

static void test_course_speedThresholdIsInclusiveAndStoppedIsRejected()
{
    auto sample = course();
    for (float speed : {0.0f, 0.99f}) {
        sample.speedMetersPerSecond = speed;
        assertRejected(sample, NavigationHeadingState::LOW_SPEED);
    }
    sample.speedMetersPerSecond = 1;
    TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW).heading.valid);
}

static void test_policy_invalidLimitsFailClosed()
{
    NavigationHeadingPolicy policy;
    TEST_ASSERT_TRUE(NavigationHeadingValidator::isPolicyValid(policy));
    policy.maxAgeMillis = 0;
    TEST_ASSERT_FALSE(NavigationHeadingValidator::isPolicyValid(policy));
    assertRejected(compass(), NavigationHeadingState::INVALID_POLICY, NOW, policy);
    policy = {};
    policy.maxCourseSpeedSkewMillis = policy.maxAgeMillis + 1;
    TEST_ASSERT_FALSE(NavigationHeadingValidator::isPolicyValid(policy));
    assertRejected(compass(), NavigationHeadingState::INVALID_POLICY, NOW, policy);
    for (float speed : {0.0f, -1.0f, NAN, INFINITY}) {
        policy = {};
        policy.minMovementSpeedMetersPerSecond = speed;
        TEST_ASSERT_FALSE(NavigationHeadingValidator::isPolicyValid(policy));
        assertRejected(course(), NavigationHeadingState::INVALID_POLICY, NOW, policy);
    }
}

static void test_policy_customLimitsApplyWithoutRememberingPreviousResult()
{
    NavigationHeadingPolicy policy{1000, 0, 3};
    auto sample = course();
    assertRejected(sample, NavigationHeadingState::LOW_SPEED, NOW, policy);
    sample.speedMetersPerSecond = 3;
    TEST_ASSERT_TRUE(NavigationHeadingValidator::evaluate(sample, NOW + 999, policy).heading.valid);
    assertRejected(sample, NavigationHeadingState::STALE, NOW + 1000, policy);
    sample.speedMeasuredAtUptimeMs--;
    assertRejected(sample, NavigationHeadingState::SPEED_UNSYNCHRONIZED, NOW, policy);
    assertRejected({}, NavigationHeadingState::NO_SAMPLE);
}

extern "C" void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_sample_defaultIsUnavailable);
    RUN_TEST(test_compass_validSamplePreservesAngleAndSource);
    RUN_TEST(test_course_validSampleDoesNotRequireCompassCalibration);
    RUN_TEST(test_sample_invalidSourceAndValidityAreRejected);
    RUN_TEST(test_angle_nonFiniteAndOutOfRangeAreRejected);
    RUN_TEST(test_time_unknownIsDifferentFromBootZero);
    RUN_TEST(test_time_futureMeasurementIsRejected);
    RUN_TEST(test_time_expiryIsInclusiveAndPreservesKnownAge);
    RUN_TEST(test_time_extendedClockDoesNotTruncateAcrossWrap);
    RUN_TEST(test_reference_requiresExplicitTrueNorth);
    RUN_TEST(test_compass_requiresCalibrationAndOrientation);
    RUN_TEST(test_course_requiresValidFixAndSpeed);
    RUN_TEST(test_course_speedTimestampMustBeKnownCurrentAndNotFuture);
    RUN_TEST(test_course_speedAndCourseSkewIsBoundedInBothDirections);
    RUN_TEST(test_course_speedThresholdIsInclusiveAndStoppedIsRejected);
    RUN_TEST(test_policy_invalidLimitsFailClosed);
    RUN_TEST(test_policy_customLimitsApplyWithoutRememberingPreviousResult);
    exit(UNITY_END());
}

extern "C" void loop() {}
