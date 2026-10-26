#include "CampPointFileStorage.h"
#include "FSCommon.h"
#include "SPILock.h"
#include <cstring>

#if defined(FSCom) && !defined(MESHTASTIC_ENCRYPTED_STORAGE)
namespace
{
constexpr const char *PATH = "/prefs/camp_point.dat";
constexpr const char *TEMP_PATH = "/prefs/camp_point.dat.tmp";

// Caller holds spiLock. Only a missing committed file means EMPTY; a bad file never becomes a target.
CampPointStatus readFile(const char *path, uint8_t *record)
{
    if (!FSCom.exists(path))
        return CampPointStatus::EMPTY;
    auto file = FSCom.open(path, FILE_O_READ);
    if (!file)
        return CampPointStatus::IO_ERROR;
    if (file.size() != CampPointStorage::RECORD_SIZE) {
        file.close();
        return CampPointStatus::CORRUPT;
    }
    const size_t count = file.read(record, CampPointStorage::RECORD_SIZE);
    file.close();
    return count == CampPointStorage::RECORD_SIZE ? CampPointStatus::OK : CampPointStatus::IO_ERROR;
}
} // namespace
#endif

CampPointStatus CampPointFileStorage::read(uint8_t *record)
{
#if defined(FSCom) && !defined(MESHTASTIC_ENCRYPTED_STORAGE)
    concurrency::LockGuard guard(spiLock);
    return readFile(PATH, record);
#else
    return CampPointStatus::UNAVAILABLE;
#endif
}

CampPointStatus CampPointFileStorage::write(const uint8_t *record)
{
#if defined(FSCom) && !defined(MESHTASTIC_ENCRYPTED_STORAGE)
    concurrency::LockGuard guard(spiLock);
    if (!FSCom.exists("/prefs") && !FSCom.mkdir("/prefs"))
        return CampPointStatus::IO_ERROR;
    // FILE_O_WRITE appends on nRF52. A leftover temporary file must be removed first.
    if (FSCom.exists(TEMP_PATH) && !FSCom.remove(TEMP_PATH))
        return CampPointStatus::IO_ERROR;
    auto file = FSCom.open(TEMP_PATH, FILE_O_WRITE);
    if (!file)
        return CampPointStatus::IO_ERROR;
    const size_t count = file.write(record, RECORD_SIZE);
    file.close();
    if (count != RECORD_SIZE)
        return CampPointStatus::IO_ERROR;
    uint8_t verified[RECORD_SIZE] = {};
    if (readFile(TEMP_PATH, verified) != CampPointStatus::OK || memcmp(record, verified, RECORD_SIZE) != 0)
        return CampPointStatus::IO_ERROR;
    // Do not delete PATH to make rename succeed. Failure preserves the previously committed file.
    // Power-loss durability of rename must still be tested on the selected board/filesystem.
    return FSCom.rename(TEMP_PATH, PATH) ? CampPointStatus::OK : CampPointStatus::IO_ERROR;
#else
    return CampPointStatus::UNAVAILABLE;
#endif
}
