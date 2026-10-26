#include "navigation/NavigationService.h"

#include "UptimeClock.h"
#include "concurrency/LockGuard.h"
#include "mesh/NodeDB.h"
#include "navigation/NavigationCalculator.h"

NavigationService *navigationService = nullptr;

namespace
{
CampActionStatus campResult(CampPointStatus status)
{
    switch (status) {
    case CampPointStatus::OK:
        return CampActionStatus::OK;
    case CampPointStatus::EMPTY:
        return CampActionStatus::NO_CAMP;
    case CampPointStatus::UNSUPPORTED_VERSION:
        return CampActionStatus::UNSUPPORTED_VERSION;
    case CampPointStatus::CORRUPT:
    case CampPointStatus::INVALID_COORDINATE:
        return CampActionStatus::CORRUPT;
    case CampPointStatus::UNAVAILABLE:
        return CampActionStatus::UNAVAILABLE;
    default:
        return CampActionStatus::STORAGE_ERROR;
    }
}
} // namespace

bool NavigationService::selectTarget(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == NODENUM_BROADCAST || nodeNum == database.getNodeNum())
        return false;
    concurrency::LockGuard guard(&mutex);
    targetNode = nodeNum;
    campTarget = false;
    return true;
}

void NavigationService::clearTarget()
{
    concurrency::LockGuard guard(&mutex);
    targetNode = 0;
    campTarget = false;
}

uint32_t NavigationService::getTarget() const
{
    concurrency::LockGuard guard(&mutex);
    return targetNode;
}

bool NavigationService::isCampTarget() const
{
    concurrency::LockGuard guard(&mutex);
    return campTarget;
}

CampActionStatus NavigationService::getCampStatus() const
{
    concurrency::LockGuard guard(&mutex);
    return campStatus;
}

CampActionStatus NavigationService::restoreCampLocked()
{
    campCoordinate = {};
    campStatus = campStore ? campResult(campStore->load(campCoordinate)) : CampActionStatus::UNAVAILABLE;
    if (campStatus != CampActionStatus::OK)
        campTarget = false;
    return campStatus;
}

CampActionStatus NavigationService::restoreCamp()
{
    concurrency::LockGuard guard(&mutex);
    return restoreCampLocked();
}

CampActionStatus NavigationService::selectCamp()
{
    concurrency::LockGuard guard(&mutex);
    if (campStatus == CampActionStatus::OK) {
        campTarget = true;
        targetNode = 0;
    }
    return campStatus;
}

CampActionStatus NavigationService::saveCamp()
{
    concurrency::LockGuard guard(&mutex);
    if (!campStore)
        return CampActionStatus::UNAVAILABLE;
    if (campStatus == CampActionStatus::UNAVAILABLE || campStatus == CampActionStatus::UNSUPPORTED_VERSION)
        return campStatus;
    meshtastic_PositionLite position{};
    PositionFreshness freshness;
    // This capture policy is independent of the more tolerant live navigation warning policy.
    const PositionFreshnessPolicy capturePolicy{30, 30};
    if (!database.copyNavigationPosition(database.getNodeNum(), position, freshness, Time::getUptimeSecs(), capturePolicy) ||
        !NavigationCalculator::isCoordinateValid({position.latitude_i, position.longitude_i}))
        return CampActionStatus::NO_LOCAL_POSITION;
    if (config.position.fixed_position || freshness.source != PositionUpdateSource::LOCAL)
        return CampActionStatus::LOCAL_POSITION_NOT_LOCAL;
    if (!freshness.ageKnown || freshness.state != PositionFreshnessState::FRESH)
        return CampActionStatus::LOCAL_POSITION_NOT_FRESH;

    const NavigationCoordinate coordinate{position.latitude_i, position.longitude_i};
    const auto result = campResult(campStore->save(coordinate));
    if (result == CampActionStatus::OK) {
        campCoordinate = coordinate;
        campStatus = CampActionStatus::OK;
    } else {
        // A failed acknowledgement may follow a successful rename. Re-read the committed file
        // instead of assuming that the previous point is still stored. Never use the .tmp.
        restoreCampLocked();
    }
    return result;
}

CampActionStatus NavigationService::deleteCamp()
{
    concurrency::LockGuard guard(&mutex);
    if (!campStore)
        return CampActionStatus::UNAVAILABLE;
    const auto result = campResult(campStore->clear());
    if (result == CampActionStatus::OK) {
        campCoordinate = {};
        campStatus = CampActionStatus::NO_CAMP;
        campTarget = false;
    } else {
        restoreCampLocked();
    }
    return result;
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
    snapshot.targetIsCamp = campTarget;
    if (!targetNode && !campTarget) {
        snapshot.positionState = NavigationPositionState::NO_TARGET_SELECTED;
        return snapshot;
    }

    const uint32_t now = Time::getUptimeSecs();
    meshtastic_PositionLite localPosition{}, targetPosition{};
    PositionFreshness localFreshness, targetFreshness;
    const bool hasLocal = database.copyNavigationPosition(database.getNodeNum(), localPosition, localFreshness, now, localPolicy);
    const bool hasTarget = campTarget
                               ? campStatus == CampActionStatus::OK
                               : database.copyNavigationPosition(targetNode, targetPosition, targetFreshness, now, targetPolicy);
    const NavigationCoordinate local{localPosition.latitude_i, localPosition.longitude_i};
    const NavigationCoordinate target =
        campTarget ? campCoordinate : NavigationCoordinate{targetPosition.latitude_i, targetPosition.longitude_i};

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
    else if (campTarget && (config.position.fixed_position || localFreshness.source != PositionUpdateSource::LOCAL))
        snapshot.positionState = NavigationPositionState::LOCAL_POSITION_NOT_LOCAL;
    else if (localFreshness.state == PositionFreshnessState::STALE)
        snapshot.positionState = NavigationPositionState::LOCAL_POSITION_STALE;
    else if (!campTarget && targetFreshness.state == PositionFreshnessState::STALE)
        snapshot.positionState = NavigationPositionState::TARGET_POSITION_STALE;
    else if (!localFreshness.ageKnown || (!campTarget && !targetFreshness.ageKnown))
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
