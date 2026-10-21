#pragma once

#include <cstdint>

enum class PositionUpdateKind : uint8_t {
    UPDATED,
    REMOVED,
    RESET,
};

enum class PositionUpdateSource : uint8_t {
    LOCAL,
    RADIO,
    USER,
    INTERNAL,
};

struct PositionUpdateEvent {
    uint32_t nodeNum = 0;
    uint32_t receivedAtUptimeSeconds = 0;
    uint32_t reportedTime = 0;
    uint8_t precisionBits = 0;
    PositionUpdateKind kind = PositionUpdateKind::UPDATED;
    PositionUpdateSource source = PositionUpdateSource::INTERNAL;
};
