#include "configuration.h"
#include "LittleFS.h"

#include <cstring>

namespace
{
constexpr uint32_t partitionStart = FLASH_BASE + GPS_WB_APP_BYTES;
constexpr uint32_t pageSize = FLASH_PAGE_SIZE;
// Keep STM32duino's last CPU1 page available for its EEPROM emulation.
constexpr uint32_t partitionEnd = FLASH_BASE + FLASH_PAGE_NUMBER * pageSize;
constexpr uint32_t pageCount = (partitionEnd - partitionStart) / pageSize;

static_assert(partitionStart < partitionEnd && GPS_WB_APP_BYTES % pageSize == 0);
static_assert(partitionEnd % pageSize == 0 && pageCount >= 4);

bool partitionAccessible()
{
    // SFSA is the first secure page. Check the whole partition before reading or writing it.
    const uint32_t firstSecurePage = (FLASH->SFR & FLASH_SFR_SFSA) >> FLASH_SFR_SFSA_Pos;
    return firstSecurePage >= (partitionEnd - FLASH_BASE) / pageSize;
}

bool validAccess(lfs_block_t block, lfs_off_t offset, lfs_size_t size)
{
    return block < pageCount && offset <= pageSize && size <= pageSize - offset;
}

int flashRead(const lfs_config *, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size)
{
    if (!validAccess(block, offset, size)) {
        return LFS_ERR_IO;
    }
    const uint32_t address = partitionStart + block * pageSize + offset;
    memcpy(buffer, reinterpret_cast<const void *>(address), size);
    return 0;
}

int flashProgram(const lfs_config *, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    if (!validAccess(block, offset, size) || offset % 8 || size % 8 || !partitionAccessible() ||
        HAL_FLASH_Unlock() != HAL_OK) {
        return LFS_ERR_IO;
    }

    const uint32_t address = partitionStart + block * pageSize + offset;
    const auto *bytes = static_cast<const uint8_t *>(buffer);
    int result = 0;
    for (lfs_size_t i = 0; i < size; i += 8) {
        uint64_t value;
        memcpy(&value, bytes + i, sizeof(value));
        if (value != UINT64_MAX && HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, value) != HAL_OK) {
            result = LFS_ERR_IO;
            break;
        }
    }
    HAL_FLASH_Lock();
    return result;
}

int flashErase(const lfs_config *, lfs_block_t block)
{
    if (block >= pageCount || !partitionAccessible() || HAL_FLASH_Unlock() != HAL_OK) {
        return LFS_ERR_IO;
    }
    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (partitionStart - FLASH_BASE) / pageSize + block;
    erase.NbPages = 1;
    uint32_t failedPage;
    const auto status = HAL_FLASHEx_Erase(&erase, &failedPage);
    HAL_FLASH_Lock();
    return status == HAL_OK ? 0 : LFS_ERR_IO;
}

int flashSync(const lfs_config *)
{
    return 0; // Erase and programming are synchronous.
}

lfs_config config = {
    nullptr, flashRead, flashProgram, flashErase, flashSync, 64, 64, pageSize, pageCount, 32, nullptr, nullptr, nullptr, nullptr,
};
} // namespace

LittleFS InternalFS;

LittleFS::LittleFS() : STM32_LittleFS(&config) {}

bool LittleFS::begin()
{
    if (!partitionAccessible()) {
        return false;
    }
    if (STM32_LittleFS::begin()) {
        return true;
    }
    // A failed mount on a previously used partition must preserve the data.
    for (uint32_t address = partitionStart; address < partitionEnd; address += sizeof(uint32_t)) {
        if (*reinterpret_cast<const volatile uint32_t *>(address) != UINT32_MAX) {
            return false;
        }
    }
    return format() && STM32_LittleFS::begin();
}
