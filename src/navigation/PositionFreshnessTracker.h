#pragma once

#include "mesh/PositionUpdateEvent.h"
#include <cstddef>
#include <cstdint>
#include <vector>

enum class PositionFreshnessState : uint8_t {
    UNKNOWN,
    FRESH,
    WARNING,
    STALE,
};

struct PositionFreshnessPolicy {
    uint32_t warningAfterSeconds = 120;
    uint32_t staleAfterSeconds = 600;
};

struct PositionFreshness {
    PositionFreshnessState state = PositionFreshnessState::UNKNOWN;
    PositionUpdateSource source = PositionUpdateSource::INTERNAL;
    uint32_t ageSeconds = 0;
    uint32_t reportedTime = 0;
    uint8_t precisionBits = 0;
    bool ageKnown = false;
};

class PositionFreshnessTracker
{
  public:
    void recordUpdate(const PositionUpdateEvent &event);
    PositionFreshness evaluate(uint32_t nodeNum, uint32_t nowUptimeSeconds, const PositionFreshnessPolicy &policy = {}) const;
    void clear();
    size_t size() const { return records.size(); }

  private:
    struct Record {
        uint32_t nodeNum;
        uint32_t receivedAtUptimeSeconds;
        uint32_t reportedTime;
        uint8_t precisionBits;
        PositionUpdateSource source;
    };

    std::vector<Record> records;
};
