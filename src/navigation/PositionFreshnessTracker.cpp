#include "navigation/PositionFreshnessTracker.h"

#include <algorithm>

void PositionFreshnessTracker::recordUpdate(const PositionUpdateEvent &event)
{
    if (event.kind == PositionUpdateKind::RESET) {
        clear();
        return;
    }

    const auto record = std::find_if(records.begin(), records.end(),
                                     [&event](const Record &candidate) { return candidate.nodeNum == event.nodeNum; });
    if (event.kind == PositionUpdateKind::REMOVED) {
        if (record != records.end())
            records.erase(record);
        return;
    }

    const Record updated{.nodeNum = event.nodeNum,
                         .receivedAtUptimeSeconds = event.receivedAtUptimeSeconds,
                         .reportedTime = event.reportedTime,
                         .precisionBits = event.precisionBits,
                         .source = event.source};
    if (record == records.end())
        records.push_back(updated);
    else
        *record = updated;
}

PositionFreshness PositionFreshnessTracker::evaluate(uint32_t nodeNum, uint32_t nowUptimeSeconds,
                                                     const PositionFreshnessPolicy &policy) const
{
    const auto record =
        std::find_if(records.begin(), records.end(), [nodeNum](const Record &candidate) { return candidate.nodeNum == nodeNum; });
    if (record == records.end() || nowUptimeSeconds < record->receivedAtUptimeSeconds)
        return {};

    PositionFreshness result;
    result.source = record->source;
    result.ageSeconds = nowUptimeSeconds - record->receivedAtUptimeSeconds;
    result.reportedTime = record->reportedTime;
    result.precisionBits = record->precisionBits;
    result.ageKnown = true;

    if (result.ageSeconds >= policy.staleAfterSeconds)
        result.state = PositionFreshnessState::STALE;
    else if (result.ageSeconds >= policy.warningAfterSeconds)
        result.state = PositionFreshnessState::WARNING;
    else
        result.state = PositionFreshnessState::FRESH;

    return result;
}

void PositionFreshnessTracker::clear()
{
    records.clear();
}
