// NavigationService in src/navigation/NavigationService.cpp joins real NodeDB positions with
// PositionFreshnessTracker and NavigationCalculator. Only current coordinate reports may drive
// navigation: telemetry, cached positions and invalid headings must not create a live arrow.
// These tests guard target-switch leakage, stale-coordinate revival, time-only refresh bugs and
// invalid/expired sensor measurements leaking through the service into NavigationPresenter.
#include "MeshTypes.h"
#include "TestUtil.h"
#include "UptimeClock.h"
#include "mesh/NodeDB.h"
#include "navigation/NavigationService.h"
#include "navigation/NavigationViewModel.h"
#include <cmath>
#include <cstdlib>
#include <memory>
#include <unity.h>

#if !MESHTASTIC_EXCLUDE_POSITIONDB
namespace
{
constexpr NodeNum TARGET_A = 0x73500001;
constexpr NodeNum TARGET_B = 0x73500002;
NodeDB *savedNodeDB;
meshtastic_DeviceState savedDeviceState;
meshtastic_NodeDatabase savedDatabase;
meshtastic_LocalConfig savedConfig;
meshtastic_LocalModuleConfig savedModuleConfig;
meshtastic_ChannelFile savedChannels;
meshtastic_Position savedPosition;
std::unique_ptr<NodeDB> database;
std::unique_ptr<NavigationService> navigation;

meshtastic_Position position(int32_t latitude = 410000000, int32_t longitude = 290000000)
{
    meshtastic_Position result{};
    result.latitude_i = latitude;
    result.longitude_i = longitude;
    result.has_latitude_i = result.has_longitude_i = true;
    result.precision_bits = 32;
    return result;
}

NavigationHeadingSample heading(float degrees = 0.0f)
{
    NavigationHeadingSample sample;
    sample.degrees = degrees;
    sample.source = NavigationHeadingSource::MAGNETOMETER;
    sample.reference = NavigationHeadingReference::TRUE_NORTH;
    sample.measuredAtUptimeMs = Time::getMillisMonotonic();
    sample.timeKnown = sample.measurementValid = sample.calibrationValid = sample.deviceOrientationValid = true;
    return sample;
}

void seedPositions()
{
    database->updatePosition(database->getNodeNum(), position(), RX_SRC_LOCAL);
    database->updatePosition(TARGET_A, position(410010000), RX_SRC_RADIO);
    TEST_ASSERT_TRUE(navigation->selectTarget(TARGET_A));
}

void assertUnavailable(NavigationPositionState expected)
{
    const auto result = navigation->getSnapshot(heading());
    TEST_ASSERT_EQUAL(expected, result.positionState);
    TEST_ASSERT_FALSE(result.arrowValid);
    TEST_ASSERT_FALSE(result.distanceValid);
    TEST_ASSERT_FALSE(result.bearingValid);
}
} // namespace

void setUp()
{
    savedNodeDB = nodeDB;
    savedDeviceState = devicestate;
    savedDatabase = nodeDatabase;
    savedConfig = config;
    savedModuleConfig = moduleConfig;
    savedChannels = channelFile;
    savedPosition = localPosition;
    nodeDatabase = {};
    database = std::make_unique<NodeDB>();
    nodeDB = database.get();
    for (const auto num : database->snapshotPositionNodeNums(0))
        database->eraseNodeSatellites(num);
    navigation = std::make_unique<NavigationService>(*database);
    Time::resetMonotonicForTests();
    Time::setTestMillis(100000);
}

void tearDown()
{
    navigation.reset();
    database.reset();
    nodeDB = savedNodeDB;
    devicestate = savedDeviceState;
    nodeDatabase = savedDatabase;
    config = savedConfig;
    moduleConfig = savedModuleConfig;
    channelFile = savedChannels;
    localPosition = savedPosition;
    Time::useRealClock();
    Time::resetMonotonicForTests();
}

static void test_selection_rejectsSentinelsAndSelfWithoutLosingTarget()
{
    assertUnavailable(NavigationPositionState::NO_TARGET_SELECTED);
    TEST_ASSERT_TRUE(navigation->selectTarget(TARGET_A));
    TEST_ASSERT_FALSE(navigation->selectTarget(0));
    TEST_ASSERT_FALSE(navigation->selectTarget(NODENUM_BROADCAST));
    TEST_ASSERT_FALSE(navigation->selectTarget(database->getNodeNum()));
    TEST_ASSERT_EQUAL_UINT32(TARGET_A, navigation->getTarget());
    navigation->clearTarget();
    assertUnavailable(NavigationPositionState::NO_TARGET_SELECTED);
}

static void test_snapshot_requiresBothPositions()
{
    navigation->selectTarget(TARGET_A);
    assertUnavailable(NavigationPositionState::NO_LOCAL_POSITION);
    database->updatePosition(database->getNodeNum(), position(), RX_SRC_LOCAL);
    assertUnavailable(NavigationPositionState::NO_TARGET_POSITION);
}

static void test_snapshot_computesDistanceAndRelativeArrow()
{
    seedPositions();
    const auto result = navigation->getSnapshot(heading(90.0f));
    TEST_ASSERT_EQUAL(NavigationPositionState::READY, result.positionState);
    TEST_ASSERT_TRUE(result.distanceValid);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 111.2f, result.distanceMeters);
    TEST_ASSERT_TRUE(result.bearingValid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, result.absoluteBearingDegrees);
    TEST_ASSERT_TRUE(result.arrowValid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -90.0f, result.relativeBearingDegrees);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::MAGNETOMETER, result.headingSource);
    TEST_ASSERT_TRUE(result.localPositionAgeKnown);
    TEST_ASSERT_TRUE(result.targetPositionAgeKnown);
    TEST_ASSERT_EQUAL_UINT8(32, result.targetPrecisionBits);
}

static void test_snapshot_neverReusesPreviousHeading()
{
    seedPositions();
    TEST_ASSERT_TRUE(navigation->getSnapshot(heading()).arrowValid);
    const auto noHeading = navigation->getSnapshot();
    TEST_ASSERT_TRUE(noHeading.distanceValid);
    TEST_ASSERT_TRUE(noHeading.bearingValid);
    TEST_ASSERT_FALSE(noHeading.arrowValid);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::NONE, noHeading.headingSource);
    TEST_ASSERT_FALSE(navigation->getSnapshot(heading(NAN)).arrowValid);
    auto movement = heading(90);
    movement.source = NavigationHeadingSource::MOVEMENT_COURSE;
    movement.fixValid = movement.speedValid = movement.speedTimeKnown = true;
    movement.speedMetersPerSecond = 2;
    movement.speedMeasuredAtUptimeMs = movement.measuredAtUptimeMs;
    TEST_ASSERT_EQUAL(NavigationHeadingSource::MOVEMENT_COURSE, navigation->getSnapshot(movement).headingSource);
}

static void test_heading_expiryHidesArrowButPreservesDistanceAndNorthBearing()
{
    seedPositions();
    const auto sample = heading(90);
    auto snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_TRUE(NavigationPresenter::present(snapshot).showArrow);
    Time::advanceTestMillis(1999);
    TEST_ASSERT_TRUE(navigation->getSnapshot(sample).arrowValid);
    Time::advanceTestMillis(1);
    snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_EQUAL(NavigationHeadingState::STALE, snapshot.headingState);
    TEST_ASSERT_EQUAL_UINT64(2000, snapshot.headingAgeMillis);
    TEST_ASSERT_TRUE(snapshot.headingAgeKnown);
    TEST_ASSERT_TRUE(snapshot.distanceValid);
    TEST_ASSERT_TRUE(snapshot.bearingValid);
    TEST_ASSERT_FALSE(snapshot.arrowValid);
    TEST_ASSERT_FALSE(NavigationPresenter::present(snapshot).showArrow);
    TEST_ASSERT_TRUE(navigation->getSnapshot(heading()).arrowValid);
}

static void test_heading_unverifiedSensorNeverEnablesArrow()
{
    seedPositions();
    auto sample = heading();
    sample.calibrationValid = false;
    auto snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_EQUAL(NavigationHeadingState::UNCALIBRATED, snapshot.headingState);
    TEST_ASSERT_FALSE(snapshot.arrowValid);
    sample.calibrationValid = true;
    sample.reference = NavigationHeadingReference::MAGNETIC_NORTH;
    snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_EQUAL(NavigationHeadingState::REFERENCE_UNVERIFIED, snapshot.headingState);
    TEST_ASSERT_FALSE(snapshot.arrowValid);
    TEST_ASSERT_TRUE(snapshot.distanceValid);
}

static void test_heading_stoppedMovementDoesNotReuseMovingArrow()
{
    seedPositions();
    auto sample = heading(90);
    sample.source = NavigationHeadingSource::MOVEMENT_COURSE;
    sample.fixValid = sample.speedValid = sample.speedTimeKnown = true;
    sample.speedMetersPerSecond = 2;
    sample.speedMeasuredAtUptimeMs = sample.measuredAtUptimeMs;
    TEST_ASSERT_TRUE(navigation->getSnapshot(sample).arrowValid);
    sample.speedMetersPerSecond = 0;
    const auto snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_EQUAL(NavigationHeadingState::LOW_SPEED, snapshot.headingState);
    TEST_ASSERT_FALSE(snapshot.arrowValid);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::NONE, snapshot.headingSource);
    TEST_ASSERT_TRUE(snapshot.distanceValid);
}

static void test_heading_invalidPolicyPreservesPreviousPolicy()
{
    seedPositions();
    NavigationHeadingPolicy policy;
    policy.maxAgeMillis = 1000;
    TEST_ASSERT_TRUE(navigation->setHeadingPolicy(policy));
    policy.maxAgeMillis = 0;
    TEST_ASSERT_FALSE(navigation->setHeadingPolicy(policy));
    const auto sample = heading();
    Time::advanceTestMillis(1000);
    TEST_ASSERT_EQUAL(NavigationHeadingState::STALE, navigation->getSnapshot(sample).headingState);
}

static void test_heading_millisWrapPreservesAgeAndExpiry()
{
    Time::setTestMillis(UINT32_MAX - 500);
    Time::serviceMonotonic();
    seedPositions();
    const auto sample = heading();
    Time::advanceTestMillis(1000);
    auto snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_TRUE(snapshot.arrowValid);
    TEST_ASSERT_EQUAL_UINT64(1000, snapshot.headingAgeMillis);
    Time::advanceTestMillis(1000);
    snapshot = navigation->getSnapshot(sample);
    TEST_ASSERT_FALSE(snapshot.arrowValid);
    TEST_ASSERT_TRUE(snapshot.distanceValid);
    TEST_ASSERT_EQUAL(NavigationHeadingState::STALE, snapshot.headingState);
    TEST_ASSERT_EQUAL_UINT64(2000, snapshot.headingAgeMillis);
}

static void test_snapshot_warningAndStaleBoundaries()
{
    seedPositions();
    Time::advanceTestMillis(119000);
    TEST_ASSERT_FALSE(navigation->getSnapshot().targetPositionWarning);
    Time::advanceTestMillis(1000);
    auto result = navigation->getSnapshot(heading());
    TEST_ASSERT_TRUE(result.localPositionWarning);
    TEST_ASSERT_TRUE(result.targetPositionWarning);
    TEST_ASSERT_TRUE(result.arrowValid);
    TEST_ASSERT_EQUAL_UINT32(120, result.targetPositionAgeSeconds);
    Time::advanceTestMillis(480000);
    assertUnavailable(NavigationPositionState::LOCAL_POSITION_STALE);
    database->updatePosition(database->getNodeNum(), position(), RX_SRC_LOCAL);
    assertUnavailable(NavigationPositionState::TARGET_POSITION_STALE);
    database->updatePosition(TARGET_A, position(410010000), RX_SRC_RADIO);
    TEST_ASSERT_TRUE(navigation->getSnapshot(heading()).arrowValid);
}

static void test_snapshot_telemetryAndTimeOnlyPacketsDoNotRefreshPosition()
{
    seedPositions();
    Time::advanceTestMillis(600000);
    database->touchNodePositionTime(TARGET_A, 1800000000);
    meshtastic_Position timeOnly{};
    timeOnly.time = 1800000100;
    database->updatePosition(TARGET_A, timeOnly, RX_SRC_USER);
    meshtastic_Telemetry telemetry{};
    telemetry.which_variant = meshtastic_Telemetry_device_metrics_tag;
    telemetry.variant.device_metrics.battery_level = 90;
    database->updateTelemetry(TARGET_A, telemetry, RX_SRC_RADIO);
    database->updatePosition(database->getNodeNum(), position(), RX_SRC_LOCAL);
    assertUnavailable(NavigationPositionState::TARGET_POSITION_STALE);
    TEST_ASSERT_EQUAL_UINT32(600, navigation->getSnapshot().targetPositionAgeSeconds);
}

static void test_selection_switchDoesNotReuseOldGeometry()
{
    seedPositions();
    TEST_ASSERT_TRUE(navigation->getSnapshot(heading()).arrowValid);
    navigation->selectTarget(TARGET_B);
    assertUnavailable(NavigationPositionState::NO_TARGET_POSITION);
    database->updatePosition(TARGET_B, position(409990000), RX_SRC_RADIO);
    const auto result = navigation->getSnapshot(heading());
    TEST_ASSERT_EQUAL_UINT32(TARGET_B, result.targetNode);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 180.0f, result.absoluteBearingDegrees);
}

static void test_snapshot_removalAndInvalidationDisableNavigation()
{
    seedPositions();
    database->eraseNodeSatellites(TARGET_A);
    assertUnavailable(NavigationPositionState::NO_TARGET_POSITION);
    seedPositions();
    database->updatePosition(TARGET_A, meshtastic_Position{}, RX_SRC_RADIO);
    assertUnavailable(NavigationPositionState::NO_TARGET_POSITION);
    TEST_ASSERT_FALSE(navigation->getSnapshot().targetPositionAgeKnown);
    seedPositions();
    database->clearLocalPosition();
    assertUnavailable(NavigationPositionState::NO_LOCAL_POSITION);
    TEST_ASSERT_FALSE(navigation->getSnapshot().localPositionAgeKnown);
}

static void test_snapshot_outOfRangeCoordinatesNeverProduceArrow()
{
    seedPositions();
    database->updatePosition(TARGET_A, position(910000000), RX_SRC_RADIO);
    assertUnavailable(NavigationPositionState::NO_TARGET_POSITION);
}

static void test_snapshot_sameLocationHasDistanceButNoDirection()
{
    seedPositions();
    database->updatePosition(TARGET_A, position(), RX_SRC_RADIO);
    const auto result = navigation->getSnapshot(heading());
    TEST_ASSERT_TRUE(result.distanceValid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result.distanceMeters);
    TEST_ASSERT_FALSE(result.bearingValid);
    TEST_ASSERT_FALSE(result.arrowValid);
}

static void test_policy_localAndTargetIntervalsAreIndependent()
{
    seedPositions();
    TEST_ASSERT_TRUE(navigation->setFreshnessPolicies({10, 30}, {60, 300}));
    TEST_ASSERT_FALSE(navigation->setFreshnessPolicies({50, 30}, {60, 300}));
    TEST_ASSERT_FALSE(navigation->setFreshnessPolicies({10, 30}, {0, 0}));
    Time::advanceTestMillis(30000);
    assertUnavailable(NavigationPositionState::LOCAL_POSITION_STALE);
    database->updatePosition(database->getNodeNum(), position(), RX_SRC_LOCAL);
    TEST_ASSERT_TRUE(navigation->getSnapshot(heading()).arrowValid);
}

static void test_snapshot_cachedCoordinatesHaveUnknownAge()
{
    seedPositions();
    // A disk-loaded satellite has coordinates but no receipt record in this boot.
    database->eraseNodeSatellites(TARGET_A);
    meshtastic_PositionLite cached{};
    cached.latitude_i = 410010000;
    cached.longitude_i = 290000000;
    cached.time = 1800000000;
    database->nodePositions[TARGET_A] = cached;
    assertUnavailable(NavigationPositionState::POSITION_AGE_UNKNOWN);
    TEST_ASSERT_FALSE(navigation->getSnapshot().targetPositionAgeKnown);
}

static void test_snapshot_millisWrapDoesNotMakePositionStale()
{
    Time::setTestMillis(UINT32_MAX - 2000);
    Time::serviceMonotonic();
    seedPositions();
    Time::advanceTestMillis(5000);
    const auto result = navigation->getSnapshot(heading());
    TEST_ASSERT_TRUE(result.arrowValid);
    TEST_ASSERT_EQUAL_UINT32(5, result.targetPositionAgeSeconds);
}

static void test_copyPosition_missingEntryClearsOutputs()
{
    seedPositions();
    meshtastic_PositionLite copied{};
    PositionFreshness freshness;
    TEST_ASSERT_TRUE(database->copyNavigationPosition(TARGET_A, copied, freshness, Time::getUptimeSecs()));
    TEST_ASSERT_TRUE(freshness.ageKnown);
    TEST_ASSERT_FALSE(database->copyNavigationPosition(TARGET_B, copied, freshness, Time::getUptimeSecs()));
    TEST_ASSERT_EQUAL_INT32(0, copied.latitude_i);
    TEST_ASSERT_FALSE(freshness.ageKnown);
}
#else
void setUp() {}
void tearDown() {}
#endif

extern "C" void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
#if !MESHTASTIC_EXCLUDE_POSITIONDB
    RUN_TEST(test_selection_rejectsSentinelsAndSelfWithoutLosingTarget);
    RUN_TEST(test_snapshot_requiresBothPositions);
    RUN_TEST(test_snapshot_computesDistanceAndRelativeArrow);
    RUN_TEST(test_snapshot_neverReusesPreviousHeading);
    RUN_TEST(test_heading_expiryHidesArrowButPreservesDistanceAndNorthBearing);
    RUN_TEST(test_heading_unverifiedSensorNeverEnablesArrow);
    RUN_TEST(test_heading_stoppedMovementDoesNotReuseMovingArrow);
    RUN_TEST(test_heading_invalidPolicyPreservesPreviousPolicy);
    RUN_TEST(test_heading_millisWrapPreservesAgeAndExpiry);
    RUN_TEST(test_snapshot_warningAndStaleBoundaries);
    RUN_TEST(test_snapshot_telemetryAndTimeOnlyPacketsDoNotRefreshPosition);
    RUN_TEST(test_selection_switchDoesNotReuseOldGeometry);
    RUN_TEST(test_snapshot_removalAndInvalidationDisableNavigation);
    RUN_TEST(test_snapshot_outOfRangeCoordinatesNeverProduceArrow);
    RUN_TEST(test_snapshot_sameLocationHasDistanceButNoDirection);
    RUN_TEST(test_policy_localAndTargetIntervalsAreIndependent);
    RUN_TEST(test_snapshot_cachedCoordinatesHaveUnknownAge);
    RUN_TEST(test_snapshot_millisWrapDoesNotMakePositionStale);
    RUN_TEST(test_copyPosition_missingEntryClearsOutputs);
#endif
    exit(UNITY_END());
}

extern "C" void loop() {}
