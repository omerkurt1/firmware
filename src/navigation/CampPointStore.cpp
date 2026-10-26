#include "CampPointStore.h"
#include "NavigationCalculator.h"
#include <ErriezCRC32.h>
#include <cstring>

namespace
{
constexpr size_t CRC_OFFSET = 16;

uint32_t read32(const uint8_t *bytes)
{
    return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
}

void write32(uint8_t *bytes, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i)
        bytes[i] = static_cast<uint8_t>(value >> (8 * i));
}

int32_t readSigned32(const uint8_t *bytes)
{
    const uint32_t value = read32(bytes);
    // Decode two's complement without an implementation-defined unsigned-to-signed cast.
    return value <= INT32_MAX ? static_cast<int32_t>(value) : static_cast<int32_t>(int64_t(value) - 0x100000000LL);
}
} // namespace

CampPointStatus CampPointStore::load(NavigationCoordinate &coordinate)
{
    coordinate = {};
    uint8_t record[CampPointStorage::RECORD_SIZE] = {};
    const auto status = storage.read(record);
    if (status != CampPointStatus::OK)
        return status;
    if (memcmp(record, "CAMP", 4) != 0 || read32(record + CRC_OFFSET) != crc32Buffer(record, CRC_OFFSET))
        return CampPointStatus::CORRUPT;
    if (record[4] != 1)
        return CampPointStatus::UNSUPPORTED_VERSION;
    if (record[5] > 1 || record[6] != 0 || record[7] != 0)
        return CampPointStatus::CORRUPT;
    NavigationCoordinate decoded{readSigned32(record + 8), readSigned32(record + 12)};
    if (record[5] == 0)
        return decoded.latitudeI == 0 && decoded.longitudeI == 0 ? CampPointStatus::EMPTY : CampPointStatus::CORRUPT;
    if (!NavigationCalculator::isCoordinateValid(decoded))
        return CampPointStatus::CORRUPT;
    coordinate = decoded;
    return CampPointStatus::OK;
}

CampPointStatus CampPointStore::save(const NavigationCoordinate &coordinate)
{
    if (!NavigationCalculator::isCoordinateValid(coordinate))
        return CampPointStatus::INVALID_COORDINATE;
    return persist(coordinate, true);
}

CampPointStatus CampPointStore::clear()
{
    // Commit an empty record through the same replacement path. Never restore a backup or .tmp.
    return persist({}, false);
}

CampPointStatus CampPointStore::persist(const NavigationCoordinate &coordinate, bool present)
{
    NavigationCoordinate previous;
    const auto previousStatus = load(previous);
    if (previousStatus != CampPointStatus::OK && previousStatus != CampPointStatus::EMPTY &&
        previousStatus != CampPointStatus::CORRUPT)
        return previousStatus;
    uint8_t record[CampPointStorage::RECORD_SIZE] = {};
    memcpy(record, "CAMP", 4);
    record[4] = 1;
    record[5] = present ? 1 : 0;
    write32(record + 8, static_cast<uint32_t>(coordinate.latitudeI));
    write32(record + 12, static_cast<uint32_t>(coordinate.longitudeI));
    write32(record + CRC_OFFSET, crc32Buffer(record, CRC_OFFSET));
    const auto status = storage.write(record);
    if (status != CampPointStatus::OK)
        return status;
    uint8_t persisted[CampPointStorage::RECORD_SIZE] = {};
    if (storage.read(persisted) != CampPointStatus::OK || memcmp(record, persisted, sizeof(record)) != 0)
        return CampPointStatus::IO_ERROR;
    return CampPointStatus::OK;
}
