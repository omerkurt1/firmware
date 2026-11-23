#include "Ht12eDecoder.h"

namespace
{
constexpr uint32_t minUnitUs = 100;
constexpr uint32_t maxUnitUs = 2000;
constexpr uint8_t wordBits = 12;
constexpr uint8_t requiredMatches = 3;

bool matchesWidth(uint32_t duration, uint32_t unit, uint32_t pulses)
{
    const uint32_t expected = pulses * unit;
    const uint32_t tolerance = unit * 45 / 100;
    return duration >= expected - tolerance && duration <= expected + tolerance;
}
} // namespace

void Ht12eDecoder::reset()
{
    *this = Ht12eDecoder{};
}

bool Ht12eDecoder::pulse(bool high, uint32_t durationUs, uint16_t &id)
{
    switch (state) {
    case State::Pilot:
        break;
    case State::Sync:
        // The pilot is 12 bit periods (36 oscillator periods), then a 1-period sync high.
        if (high && durationUs >= minUnitUs && durationUs <= maxUnitUs &&
            pilotUs >= 27 * durationUs && pilotUs <= 45 * durationUs) {
            unitUs = (pilotUs + 18) / 36;
            word = 0;
            bits = 0;
            state = State::DataLow;
            return false;
        }
        break;
    case State::DataLow:
        if (!high) {
            const bool zero = matchesWidth(durationUs, unitUs, 1);
            const bool one = matchesWidth(durationUs, unitUs, 2);
            if (zero || one) {
                dataBit = one;
                state = State::DataHigh;
                return false;
            }
        }
        break;
    case State::DataHigh:
        if (high && matchesWidth(durationUs, unitUs, dataBit ? 1 : 2)) {
            word = static_cast<uint16_t>((word << 1) | dataBit);
            if (++bits < wordBits) {
                state = State::DataLow;
                return false;
            }
            state = State::Pilot;
            matches = word == previousWord ? matches + 1 : 1;
            previousWord = word;
            if (matches == requiredMatches) {
                id = word;
                matches = 0;
                return true;
            }
            return false;
        }
        break;
    }

    const bool validPilot = !high && durationUs >= 27 * minUnitUs && durationUs <= 45 * maxUnitUs;
    if (state != State::Pilot || !validPilot) {
        reset();
    }
    if (validPilot) {
        pilotUs = durationUs;
        state = State::Sync;
    }
    return false;
}
