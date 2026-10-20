#include "MeshTypes.h"
#include "TestUtil.h"
#include "navigation/NavigationCalculator.h"
#include <cmath>
#include <unity.h>

// These tests pin NavigationCalculator's coordinate validation, geodesic distance, absolute bearing,
// and device-relative bearing contracts. They guard against unsafe UI regressions such as treating
// Meshtastic's unknown 0,0 position as a real fix or rotating an arrow the long way around 0/360 degrees.

namespace
{
constexpr float DISTANCE_TOLERANCE_METERS = 600.0f;
constexpr float BEARING_TOLERANCE_DEGREES = 0.2f;

NavigationCoordinate coordinate(double latitude, double longitude)
{
    return NavigationCoordinate{.latitudeI = static_cast<int32_t>(latitude * 1e7),
                                .longitudeI = static_cast<int32_t>(longitude * 1e7)};
}
} // namespace

static void test_coordinateValidation_rejectsUnknownAndOutOfRangeValues()
{
    TEST_ASSERT_FALSE(NavigationCalculator::isCoordinateValid(coordinate(0.0, 0.0)));
    TEST_ASSERT_FALSE(NavigationCalculator::isCoordinateValid(coordinate(90.1, 10.0)));
    TEST_ASSERT_FALSE(NavigationCalculator::isCoordinateValid(coordinate(10.0, 180.1)));
}

static void test_coordinateValidation_acceptsEquatorAndPrimeMeridianPositions()
{
    TEST_ASSERT_TRUE(NavigationCalculator::isCoordinateValid(coordinate(0.0, 10.0)));
    TEST_ASSERT_TRUE(NavigationCalculator::isCoordinateValid(coordinate(10.0, 0.0)));
    TEST_ASSERT_TRUE(NavigationCalculator::isCoordinateValid(coordinate(-90.0, -180.0)));
}

static void test_normalizeDegrees360_wrapsPositiveAndNegativeAngles()
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, NavigationCalculator::normalizeDegrees360(361.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 359.0f, NavigationCalculator::normalizeDegrees360(-1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, NavigationCalculator::normalizeDegrees360(720.0f));
}

static void test_normalizeDeltaDegrees180_usesShortestTurn()
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, NavigationCalculator::normalizeDeltaDegrees180(20.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, NavigationCalculator::normalizeDeltaDegrees180(340.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, NavigationCalculator::normalizeDeltaDegrees180(1.0f - 359.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, NavigationCalculator::normalizeDeltaDegrees180(359.0f - 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -180.0f, NavigationCalculator::normalizeDeltaDegrees180(180.0f));
}

static void test_calculate_rejectsInvalidOriginOrTarget()
{
    const NavigationGeometry invalidOrigin = NavigationCalculator::calculate(coordinate(0.0, 0.0), coordinate(1.0, 2.0));
    const NavigationGeometry invalidTarget = NavigationCalculator::calculate(coordinate(1.0, 2.0), coordinate(0.0, 0.0));

    TEST_ASSERT_FALSE(invalidOrigin.distanceValid);
    TEST_ASSERT_FALSE(invalidOrigin.bearingValid);
    TEST_ASSERT_FALSE(invalidTarget.distanceValid);
    TEST_ASSERT_FALSE(invalidTarget.bearingValid);
}

static void test_calculate_samePointHasDistanceButNoDirection()
{
    const NavigationCoordinate point = coordinate(41.0082, 28.9784);
    const NavigationGeometry result = NavigationCalculator::calculate(point, point);

    TEST_ASSERT_TRUE(result.distanceValid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result.distanceMeters);
    TEST_ASSERT_FALSE(result.bearingValid);
    TEST_ASSERT_FALSE(result.relativeBearingValid);
}

static void test_calculate_oneLongitudeDegreeAtEquatorIsAbout111Kilometers()
{
    const NavigationGeometry result = NavigationCalculator::calculate(coordinate(0.0, 1.0), coordinate(0.0, 2.0));

    TEST_ASSERT_TRUE(result.distanceValid);
    TEST_ASSERT_FLOAT_WITHIN(DISTANCE_TOLERANCE_METERS, 111107.0f, result.distanceMeters);
}

static void test_calculate_returnsCardinalAbsoluteBearings()
{
    const NavigationCoordinate origin = coordinate(10.0, 10.0);
    const NavigationGeometry north = NavigationCalculator::calculate(origin, coordinate(11.0, 10.0));
    const NavigationGeometry east = NavigationCalculator::calculate(origin, coordinate(10.0, 11.0));
    const NavigationGeometry south = NavigationCalculator::calculate(origin, coordinate(9.0, 10.0));
    const NavigationGeometry west = NavigationCalculator::calculate(origin, coordinate(10.0, 9.0));

    TEST_ASSERT_FLOAT_WITHIN(BEARING_TOLERANCE_DEGREES, 0.0f, north.absoluteBearingDegrees);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_TOLERANCE_DEGREES, 89.91f, east.absoluteBearingDegrees);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_TOLERANCE_DEGREES, 180.0f, south.absoluteBearingDegrees);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_TOLERANCE_DEGREES, 270.09f, west.absoluteBearingDegrees);
}

static void test_calculate_relativeBearingCrossesNorthByShortestPath()
{
    NavigationHeading heading{.degrees = 350.0f, .source = NavigationHeadingSource::MAGNETOMETER, .valid = true};
    const NavigationGeometry result = NavigationCalculator::calculate(coordinate(10.0, 10.0), coordinate(11.0, 10.0), heading);

    TEST_ASSERT_TRUE(result.relativeBearingValid);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_TOLERANCE_DEGREES, 10.0f, result.relativeBearingDegrees);
}

static void test_calculate_doesNotProduceRelativeBearingWithoutUsableHeading()
{
    NavigationHeading missingHeading{.degrees = 20.0f, .source = NavigationHeadingSource::NONE, .valid = true};
    const NavigationGeometry result =
        NavigationCalculator::calculate(coordinate(10.0, 10.0), coordinate(11.0, 10.0), missingHeading);

    TEST_ASSERT_TRUE(result.bearingValid);
    TEST_ASSERT_FALSE(result.relativeBearingValid);
}

void setUp(void) {}

void tearDown(void) {}

extern "C" {
void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_coordinateValidation_rejectsUnknownAndOutOfRangeValues);
    RUN_TEST(test_coordinateValidation_acceptsEquatorAndPrimeMeridianPositions);
    RUN_TEST(test_normalizeDegrees360_wrapsPositiveAndNegativeAngles);
    RUN_TEST(test_normalizeDeltaDegrees180_usesShortestTurn);
    RUN_TEST(test_calculate_rejectsInvalidOriginOrTarget);
    RUN_TEST(test_calculate_samePointHasDistanceButNoDirection);
    RUN_TEST(test_calculate_oneLongitudeDegreeAtEquatorIsAbout111Kilometers);
    RUN_TEST(test_calculate_returnsCardinalAbsoluteBearings);
    RUN_TEST(test_calculate_relativeBearingCrossesNorthByShortestPath);
    RUN_TEST(test_calculate_doesNotProduceRelativeBearingWithoutUsableHeading);
    exit(UNITY_END());
}

void loop() {}
}
