#pragma once

#include "NavigationTypes.h"
#include <cstddef>
#include <cstdint>

enum class CampPointStatus : uint8_t {
    OK,
    EMPTY,
    INVALID_COORDINATE,
    CORRUPT,
    UNSUPPORTED_VERSION,
    IO_ERROR,
    UNAVAILABLE,
};

// Fixed-size record I/O. Implementations must reject short/oversized reads and preserve the
// committed file until the replacement has been fully written and verified.
class CampPointStorage
{
  public:
    static constexpr size_t RECORD_SIZE = 20;
    virtual ~CampPointStorage() = default;
    virtual CampPointStatus read(uint8_t *record) = 0;
    virtual CampPointStatus write(const uint8_t *record) = 0;
};

// Single-owner synchronous storage, called after filesystem initialization, never from an ISR.
// No cached target survives a failed load. save() validates coordinates, not GNSS freshness:
// the caller must obtain a fresh local fix before offering "save current location".
class CampPointStore
{
  public:
    explicit CampPointStore(CampPointStorage &storage) : storage(storage) {}
    CampPointStatus load(NavigationCoordinate &coordinate);
    CampPointStatus save(const NavigationCoordinate &coordinate);
    CampPointStatus clear();

  private:
    CampPointStatus persist(const NavigationCoordinate &coordinate, bool present);
    CampPointStorage &storage;
};
