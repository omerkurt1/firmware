#pragma once

#include <cstdint>

// Holtek HT12E, datasheet pp. 5–7. Each data bit begins with the low portion.
class Ht12eDecoder
{
  public:
    // Feed a completed pulse in wire polarity. Return true after three matching words.
    bool pulse(bool high, uint32_t durationUs, uint16_t &id);
    void reset();

  private:
    enum class State : uint8_t { Pilot, Sync, DataLow, DataHigh };
    State state = State::Pilot;
    uint32_t pilotUs = 0;
    uint32_t unitUs = 0;
    uint16_t word = 0;
    uint16_t previousWord = 0;
    uint8_t bits = 0;
    uint8_t matches = 0;
    bool dataBit = false;
};
