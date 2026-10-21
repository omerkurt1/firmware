#include "MeshTypes.h"
#include "TestUtil.h"
#include "navigation/PositionFreshnessTracker.h"
#include <cstdlib>
#include <unity.h>

// These tests pin PositionFreshnessTracker's per-position monotonic age contract and lifecycle handling.
// They guard against the navigation arrow becoming fresh from unrelated traffic, stale records surviving
// node removal/reset, and a clock rollback being interpreted as a recent position.

namespace
{
constexpr uint32_t NODE_A = 0x12345678;
constexpr uint32_t NODE_B = 0x87654321;

PositionFreshnessTracker tracker;
PositionFreshnessPolicy policy{.warningAfterSeconds = 120, .staleAfterSeconds = 600};

PositionUpdateEvent update(uint32_t nodeNum, uint32_t uptimeSeconds, PositionUpdateSource source = PositionUpdateSource::RADIO)
{
    return PositionUpdateEvent{.nodeNum = nodeNum,
                               .receivedAtUptimeSeconds = uptimeSeconds,
                               .reportedTime = 1700000000,
                               .precisionBits = 24,
                               .kind = PositionUpdateKind::UPDATED,
                               .source = source};
}
} // namespace

static void test_evaluate_unknownNodeHasUnknownAge()
{
    const PositionFreshness result = tracker.evaluate(NODE_A, 100, policy);

    TEST_ASSERT_EQUAL(PositionFreshnessState::UNKNOWN, result.state);
    TEST_ASSERT_FALSE(result.ageKnown);
}

static void test_evaluate_classifiesFreshWarningAndStaleBoundaries()
{
    tracker.recordUpdate(update(NODE_A, 100));

    TEST_ASSERT_EQUAL(PositionFreshnessState::FRESH, tracker.evaluate(NODE_A, 219, policy).state);
    TEST_ASSERT_EQUAL(PositionFreshnessState::WARNING, tracker.evaluate(NODE_A, 220, policy).state);
    TEST_ASSERT_EQUAL(PositionFreshnessState::WARNING, tracker.evaluate(NODE_A, 699, policy).state);
    TEST_ASSERT_EQUAL(PositionFreshnessState::STALE, tracker.evaluate(NODE_A, 700, policy).state);
}

static void test_evaluate_preservesSourceTimeAndPrecision()
{
    tracker.recordUpdate(update(NODE_A, 100, PositionUpdateSource::USER));
    const PositionFreshness result = tracker.evaluate(NODE_A, 145, policy);

    TEST_ASSERT_TRUE(result.ageKnown);
    TEST_ASSERT_EQUAL_UINT32(45, result.ageSeconds);
    TEST_ASSERT_EQUAL_UINT32(1700000000, result.reportedTime);
    TEST_ASSERT_EQUAL_UINT8(24, result.precisionBits);
    TEST_ASSERT_EQUAL(PositionUpdateSource::USER, result.source);
}

static void test_recordUpdate_sameNodeRefreshesWithoutDuplicateRecord()
{
    tracker.recordUpdate(update(NODE_A, 100));
    tracker.recordUpdate(update(NODE_A, 500));

    TEST_ASSERT_EQUAL_UINT(1, tracker.size());
    TEST_ASSERT_EQUAL_UINT32(10, tracker.evaluate(NODE_A, 510, policy).ageSeconds);
}

static void test_recordUpdate_otherNodeDoesNotRefreshTarget()
{
    tracker.recordUpdate(update(NODE_A, 100));
    tracker.recordUpdate(update(NODE_B, 500));

    TEST_ASSERT_EQUAL_UINT32(500, tracker.evaluate(NODE_A, 600, policy).ageSeconds);
    TEST_ASSERT_EQUAL_UINT32(100, tracker.evaluate(NODE_B, 600, policy).ageSeconds);
}

static void test_recordUpdate_removedNodeForgetsItsAge()
{
    tracker.recordUpdate(update(NODE_A, 100));
    tracker.recordUpdate(PositionUpdateEvent{.nodeNum = NODE_A, .kind = PositionUpdateKind::REMOVED});

    TEST_ASSERT_EQUAL_UINT(0, tracker.size());
    TEST_ASSERT_FALSE(tracker.evaluate(NODE_A, 150, policy).ageKnown);
}

static void test_recordUpdate_resetForgetsEveryNode()
{
    tracker.recordUpdate(update(NODE_A, 100));
    tracker.recordUpdate(update(NODE_B, 110));
    tracker.recordUpdate(PositionUpdateEvent{.kind = PositionUpdateKind::RESET});

    TEST_ASSERT_EQUAL_UINT(0, tracker.size());
    TEST_ASSERT_FALSE(tracker.evaluate(NODE_A, 150, policy).ageKnown);
    TEST_ASSERT_FALSE(tracker.evaluate(NODE_B, 150, policy).ageKnown);
}

static void test_evaluate_clockRollbackIsUnknownInsteadOfFresh()
{
    tracker.recordUpdate(update(NODE_A, 500));
    const PositionFreshness result = tracker.evaluate(NODE_A, 499, policy);

    TEST_ASSERT_EQUAL(PositionFreshnessState::UNKNOWN, result.state);
    TEST_ASSERT_FALSE(result.ageKnown);
}

void setUp(void)
{
    tracker.clear();
}

void tearDown(void) {}

extern "C" {
void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_evaluate_unknownNodeHasUnknownAge);
    RUN_TEST(test_evaluate_classifiesFreshWarningAndStaleBoundaries);
    RUN_TEST(test_evaluate_preservesSourceTimeAndPrecision);
    RUN_TEST(test_recordUpdate_sameNodeRefreshesWithoutDuplicateRecord);
    RUN_TEST(test_recordUpdate_otherNodeDoesNotRefreshTarget);
    RUN_TEST(test_recordUpdate_removedNodeForgetsItsAge);
    RUN_TEST(test_recordUpdate_resetForgetsEveryNode);
    RUN_TEST(test_evaluate_clockRollbackIsUnknownInsteadOfFresh);
    exit(UNITY_END());
}

void loop() {}
}
