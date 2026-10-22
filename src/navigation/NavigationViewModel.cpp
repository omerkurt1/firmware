#include "navigation/NavigationViewModel.h"

#include <cmath>
#include <cstdio>

namespace
{
void formatAge(char *out, size_t size, bool known, uint32_t seconds)
{
    if (!known)
        snprintf(out, size, "?");
    else if (seconds < 60)
        snprintf(out, size, "%lus", static_cast<unsigned long>(seconds));
    else if (seconds < 3600)
        snprintf(out, size, "%lum", static_cast<unsigned long>(seconds / 60));
    else if (seconds < 86400)
        snprintf(out, size, "%luh", static_cast<unsigned long>(seconds / 3600));
    else
        snprintf(out, size, "%lud", static_cast<unsigned long>(seconds / 86400));
}
} // namespace

NavigationViewModel NavigationPresenter::present(const NavigationSnapshot &snapshot, bool imperial)
{
    NavigationViewModel view;
    view.targetNode = snapshot.targetNode;
    if (snapshot.positionState == NavigationPositionState::NO_TARGET_SELECTED) {
        snprintf(view.primary, sizeof(view.primary), "No target");
        snprintf(view.detail, sizeof(view.detail), "Hold: select target");
        return view;
    }

    char localAge[16], targetAge[16];
    formatAge(localAge, sizeof(localAge), snapshot.localPositionAgeKnown, snapshot.localPositionAgeSeconds);
    formatAge(targetAge, sizeof(targetAge), snapshot.targetPositionAgeKnown, snapshot.targetPositionAgeSeconds);
    snprintf(view.ages, sizeof(view.ages), "Target:%s  Me:%s", targetAge, localAge);

    const char *primary = nullptr;
    const char *detail = "Wait for position";
    switch (snapshot.positionState) {
    case NavigationPositionState::NO_LOCAL_POSITION:
        primary = "No local position";
        detail = "Check GPS / sky view";
        break;
    case NavigationPositionState::NO_TARGET_POSITION:
        primary = "No target position";
        detail = "Check target / range";
        break;
    case NavigationPositionState::LOCAL_POSITION_STALE:
        primary = "Local position old";
        detail = "Check GPS / sky view";
        break;
    case NavigationPositionState::TARGET_POSITION_STALE:
        primary = "Target position old";
        detail = "Check target / range";
        break;
    case NavigationPositionState::POSITION_AGE_UNKNOWN:
        primary = "Position age unknown";
        break;
    case NavigationPositionState::READY:
        break;
    default:
        primary = "Navigation unavailable";
        break;
    }
    if (!primary && (!snapshot.distanceValid || !std::isfinite(snapshot.distanceMeters) || snapshot.distanceMeters < 0))
        primary = "Distance unavailable";
    if (primary) {
        snprintf(view.primary, sizeof(view.primary), "%s", primary);
        snprintf(view.detail, sizeof(view.detail), "%s", detail);
        return view;
    }

    view.showDistance = true;
    const double distance = snapshot.distanceMeters;
    if (imperial) {
        const double feet = distance * 3.280839895;
        if (feet < 5280)
            snprintf(view.primary, sizeof(view.primary), "%.0f ft", feet);
        else
            snprintf(view.primary, sizeof(view.primary), "%.1f mi", feet / 5280);
    } else if (distance < 1000) {
        snprintf(view.primary, sizeof(view.primary), "%.0f m", distance);
    } else {
        snprintf(view.primary, sizeof(view.primary), "%.1f km", distance / 1000);
    }

    view.showArrow = distance > 0 && snapshot.arrowValid && snapshot.bearingValid &&
                     std::isfinite(snapshot.relativeBearingDegrees) && snapshot.headingSource != NavigationHeadingSource::NONE;
    if (view.showArrow)
        view.arrowDegrees = snapshot.relativeBearingDegrees;

    if (snapshot.localPositionWarning && snapshot.targetPositionWarning)
        snprintf(view.detail, sizeof(view.detail), "Both positions aging");
    else if (snapshot.localPositionWarning)
        snprintf(view.detail, sizeof(view.detail), "Local position aging");
    else if (snapshot.targetPositionWarning)
        snprintf(view.detail, sizeof(view.detail), "Target position aging");
    else if (distance == 0)
        snprintf(view.detail, sizeof(view.detail), "Same coordinates");
    else if (view.showArrow)
        snprintf(view.detail, sizeof(view.detail), "%s",
                 snapshot.headingSource == NavigationHeadingSource::MAGNETOMETER ? "Compass direction" : "Movement direction");
    else if (snapshot.bearingValid && std::isfinite(snapshot.absoluteBearingDegrees))
        snprintf(view.detail, sizeof(view.detail), "N: %.0f deg / no arrow", snapshot.absoluteBearingDegrees);
    else
        snprintf(view.detail, sizeof(view.detail), "Direction unavailable");
    return view;
}
