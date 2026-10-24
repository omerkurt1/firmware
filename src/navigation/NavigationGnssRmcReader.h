#pragma once

#include "navigation/NavigationHeadingValidator.h"
#include <cstddef>

struct NavigationGnssObservation {
    // Coherent course/speed, but measurement time is NOT established by NMEA receipt.
    NavigationHeadingSample heading;
    uint64_t receivedAtUptimeMs = 0;
    bool received = false;
};

class NavigationGnssRmcReader
{
  public:
    // Caller serializes encode/reset/copyObservation. No heap allocation or hardware access.
    void encode(char byte, uint64_t receivedAtUptimeMs);
    void reset();
    NavigationGnssObservation copyObservation() const { return observation; }

  private:
    void finish(uint64_t receivedAtUptimeMs);
    char buffer[128] = {};
    size_t length = 0;
    bool collecting = false;
    NavigationGnssObservation observation;
};
