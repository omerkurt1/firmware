#include "navigation/NavigationService.h"

#include "UptimeClock.h"
#include "concurrency/LockGuard.h"
#include "mesh/NodeDB.h"
#include "navigation/NavigationCalculator.h"

NavigationService *navigationService = nullptr;

bool NavigationService::selectTarget(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == NODENUM_BROADCAST || nodeNum == database.getNodeNum())
        return false;
    concurrency::LockGuard guard(&mutex);
    targetNode = nodeNum;
    return true;
}

void NavigationService::clearTarget()
{
    concurrency::LockGuard guard(&mutex);
    targetNode = 0;
}

uint32_t NavigationService::getTarget() const
{
    concurrency::LockGuard guard(&mutex);
    return targetNode;
}

bool NavigationService::setFreshnessPolicies(const PositionFreshnessPolicy &local, const PositionFreshnessPolicy &target)
{
    if (local.staleAfterSeconds == 0 || target.staleAfterSeconds == 0 || local.warningAfterSeconds > local.staleAfterSeconds ||
        target.warningAfterSeconds > target.staleAfterSeconds)
        return false;
    concurrency::LockGuard guard(&mutex);
    localPolicy = local;
    targetPolicy = target;
    return true;
}

bool NavigationService::setHeadingPolicy(const NavigationHeadingPolicy &policy)
{
    if (!NavigationHeadingValidator::isPolicyValid(policy))
        return false;
    concurrency::LockGuard guard(&mutex);
    headingPolicy = policy;
    return true;
}

NavigationSnapshot NavigationService::getSnapshot(const NavigationHeadingSample &heading) const
{
    concurrency::LockGuard guard(&mutex);
    NavigationSnapshot snapshot;
    const auto evaluatedHeading = NavigationHeadingValidator::evaluate(heading, Time::getMillisMonotonic(), headingPolicy);
    snapshot.headingState = evaluatedHeading.state;
    snapshot.headingAgeKnown = evaluatedHeading.ageKnown;
    snapshot.headingAgeMillis = evaluatedHeading.ageMillis;
    snapshot.targetNode = targetNode;
    if (!targetNode) {
        snapshot.positionState = NavigationPositionState::NO_TARGET_SELECTED;
        return snapshot;
    }

    const uint32_t now = Time::getUptimeSecs();
    meshtastic_PositionLite localPosition{}, targetPosition{};
    PositionFreshness localFreshness, targetFreshness;
    const bool hasLocal = database.copyNavigationPosition(database.getNodeNum(), localPosition, localFreshness, now, localPolicy);
    const bool hasTarget = database.copyNavigationPosition(targetNode, targetPosition, targetFreshness, now, targetPolicy);
    const NavigationCoordinate local{localPosition.latitude_i, localPosition.longitude_i};
    const NavigationCoordinate target{targetPosition.latitude_i, targetPosition.longitude_i};

    snapshot.localPositionAgeSeconds = localFreshness.ageSeconds;
    snapshot.targetPositionAgeSeconds = targetFreshness.ageSeconds;
    snapshot.localPositionAgeKnown = localFreshness.ageKnown;
    snapshot.targetPositionAgeKnown = targetFreshness.ageKnown;
    snapshot.targetPrecisionBits = targetFreshness.precisionBits;
    snapshot.localPositionWarning = localFreshness.state == PositionFreshnessState::WARNING;
    snapshot.targetPositionWarning = targetFreshness.state == PositionFreshnessState::WARNING;

    if (!hasLocal || !NavigationCalculator::isCoordinateValid(local))
        snapshot.positionState = NavigationPositionState::NO_LOCAL_POSITION;
    else if (!hasTarget || !NavigationCalculator::isCoordinateValid(target))
        snapshot.positionState = NavigationPositionState::NO_TARGET_POSITION;
    else if (localFreshness.state == PositionFreshnessState::STALE)
        snapshot.positionState = NavigationPositionState::LOCAL_POSITION_STALE;
    else if (targetFreshness.state == PositionFreshnessState::STALE)
        snapshot.positionState = NavigationPositionState::TARGET_POSITION_STALE;
    else if (!localFreshness.ageKnown || !targetFreshness.ageKnown)
        snapshot.positionState = NavigationPositionState::POSITION_AGE_UNKNOWN;
    else
        snapshot.positionState = NavigationPositionState::READY;

    if (snapshot.positionState != NavigationPositionState::READY)
        return snapshot;

    const auto geometry = NavigationCalculator::calculate(local, target, evaluatedHeading.heading);
    snapshot.distanceMeters = geometry.distanceMeters;
    snapshot.absoluteBearingDegrees = geometry.absoluteBearingDegrees;
    snapshot.relativeBearingDegrees = geometry.relativeBearingDegrees;
    snapshot.distanceValid = geometry.distanceValid;
    snapshot.bearingValid = geometry.bearingValid;
    snapshot.arrowValid = geometry.relativeBearingValid;
    if (snapshot.arrowValid)
        snapshot.headingSource = evaluatedHeading.heading.source;
    return snapshot;
}
