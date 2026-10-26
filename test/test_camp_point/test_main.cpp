// CampPointStore / CampPointFileStorage: restart persistence, explicit deletion and fail-closed
// loading. Fault injection covers write/readback failures; real FSCom tests cover staging and
// replacement. Host tests do not establish physical flash durability on power loss.
#include "FSCommon.h"
#include "MeshTypes.h"
#include "TestUtil.h"
#include "navigation/CampPointFileStorage.h"
#include <ErriezCRC32.h>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <unity.h>
#if defined(ARCH_PORTDUINO) && !defined(_WIN32)
#include <string>
#include <unistd.h>
#endif

namespace
{
constexpr NavigationCoordinate CAMP{-410000000, 290000000};
constexpr NavigationCoordinate OTHER{410000000, -290000000};
constexpr const char *PATH = "/prefs/camp_point.dat";
constexpr const char *TEMP_PATH = "/prefs/camp_point.dat.tmp";
// Independently generated with Python struct.pack('<4sBBBBii', ...) + zlib.crc32.
constexpr uint8_t GOLDEN[] = {0x43, 0x41, 0x4d, 0x50, 0x01, 0x01, 0x00, 0x00, 0x80, 0xe5,
                              0x8f, 0xe7, 0x80, 0x0c, 0x49, 0x11, 0xf5, 0xa0, 0x16, 0xe5};

class MemoryStorage : public CampPointStorage
{
  public:
    uint8_t bytes[RECORD_SIZE] = {};
    CampPointStatus readStatus = CampPointStatus::EMPTY;
    CampPointStatus writeStatus = CampPointStatus::OK;
    bool discardWrite = false;
    bool damageWrite = false;
    bool failReadback = false;
    size_t writes = 0;

    CampPointStatus read(uint8_t *record) override
    {
        if (readStatus == CampPointStatus::OK)
            memcpy(record, bytes, RECORD_SIZE);
        return readStatus;
    }
    CampPointStatus write(const uint8_t *record) override
    {
        ++writes;
        if (writeStatus != CampPointStatus::OK)
            return writeStatus;
        if (!discardWrite) {
            memcpy(bytes, record, RECORD_SIZE);
            readStatus = CampPointStatus::OK;
        }
        if (damageWrite)
            bytes[10] ^= 0x80;
        if (failReadback)
            readStatus = CampPointStatus::IO_ERROR;
        return CampPointStatus::OK;
    }
};
MemoryStorage memory;
CampPointFileStorage files;

void assertLoad(CampPointStore &store, CampPointStatus expected, NavigationCoordinate expectedCoordinate = {})
{
    NavigationCoordinate out = OTHER;
    TEST_ASSERT_EQUAL(expected, store.load(out));
    TEST_ASSERT_EQUAL_INT32(expectedCoordinate.latitudeI, out.latitudeI);
    TEST_ASSERT_EQUAL_INT32(expectedCoordinate.longitudeI, out.longitudeI);
}

void useGolden()
{
    memcpy(memory.bytes, GOLDEN, sizeof(GOLDEN));
    memory.readStatus = CampPointStatus::OK;
}

void refreshCrc()
{
    const uint32_t crc = crc32Buffer(memory.bytes, 16);
    for (size_t i = 0; i < 4; ++i)
        memory.bytes[16 + i] = static_cast<uint8_t>(crc >> (8 * i));
}

void removeFile(const char *path)
{
    if (FSCom.exists(path))
        TEST_ASSERT_TRUE(FSCom.remove(path));
}

void writeRaw(const char *path, const uint8_t *bytes, size_t size)
{
    removeFile(path);
    auto file = FSCom.open(path, FILE_O_WRITE);
    TEST_ASSERT_TRUE(file);
    TEST_ASSERT_EQUAL(size, file.write(bytes, size));
    file.close();
}
} // namespace

void setUp()
{
    memory = MemoryStorage{};
    if (!FSCom.exists("/prefs"))
        TEST_ASSERT_TRUE(FSCom.mkdir("/prefs"));
    removeFile(PATH);
    removeFile(TEMP_PATH);
}
void tearDown() {}

static void test_empty_or_unavailable_load_clears_output()
{
    CampPointStore store(memory);
    for (const auto status : {CampPointStatus::EMPTY, CampPointStatus::IO_ERROR, CampPointStatus::UNAVAILABLE}) {
        memory.readStatus = status;
        assertLoad(store, status);
    }
}

static void test_golden_record_and_signed_coordinate_round_trip()
{
    CampPointStore store(memory);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(CAMP));
    TEST_ASSERT_EQUAL_MEMORY(GOLDEN, memory.bytes, sizeof(GOLDEN));
    CampPointStore restarted(memory);
    assertLoad(restarted, CampPointStatus::OK, CAMP);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.save(OTHER));
    assertLoad(store, CampPointStatus::OK, OTHER);
}

static void test_invalid_coordinates_do_not_replace_saved_point()
{
    useGolden();
    CampPointStore store(memory);
    for (const auto coordinate :
         {NavigationCoordinate{0, 0}, NavigationCoordinate{900000001, 1}, NavigationCoordinate{-900000001, 1},
          NavigationCoordinate{1, 1800000001}, NavigationCoordinate{1, -1800000001}})
        TEST_ASSERT_EQUAL(CampPointStatus::INVALID_COORDINATE, store.save(coordinate));
    TEST_ASSERT_EQUAL(0, memory.writes);
    assertLoad(store, CampPointStatus::OK, CAMP);
}

static void test_boundary_coordinates_are_preserved()
{
    CampPointStore store(memory);
    for (const auto coordinate : {NavigationCoordinate{900000000, 1800000000}, NavigationCoordinate{-900000000, -1800000000},
                                  NavigationCoordinate{0, 290000000}, NavigationCoordinate{410000000, 0}}) {
        TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(coordinate));
        assertLoad(store, CampPointStatus::OK, coordinate);
    }
}

static void test_clear_survives_store_reconstruction_and_is_repeatable()
{
    useGolden();
    CampPointStore store(memory);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.clear());
    CampPointStore restarted(memory);
    assertLoad(restarted, CampPointStatus::EMPTY);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.clear());
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.save(OTHER));
    assertLoad(restarted, CampPointStatus::OK, OTHER);
}

static void test_every_single_byte_corruption_clears_output()
{
    CampPointStore store(memory);
    for (size_t i = 0; i < sizeof(GOLDEN); ++i) {
        useGolden();
        memory.bytes[i] ^= 0x01;
        assertLoad(store, CampPointStatus::CORRUPT);
    }
}

static void test_unknown_version_is_reported_and_not_overwritten()
{
    useGolden();
    memory.bytes[4] = 2;
    refreshCrc();
    CampPointStore store(memory);
    assertLoad(store, CampPointStatus::UNSUPPORTED_VERSION);
    TEST_ASSERT_EQUAL(CampPointStatus::UNSUPPORTED_VERSION, store.save(OTHER));
    TEST_ASSERT_EQUAL(CampPointStatus::UNSUPPORTED_VERSION, store.clear());
    TEST_ASSERT_EQUAL(0, memory.writes);
}

static void test_valid_crc_does_not_bypass_schema_checks()
{
    CampPointStore store(memory);
    for (const size_t offset : {size_t(0), size_t(5), size_t(6), size_t(7)}) {
        useGolden();
        memory.bytes[offset] = 3;
        refreshCrc();
        assertLoad(store, CampPointStatus::CORRUPT);
    }
    useGolden();
    memory.bytes[5] = 0; // A tombstone must not contain coordinates.
    refreshCrc();
    assertLoad(store, CampPointStatus::CORRUPT);
    useGolden();
    memset(memory.bytes + 8, 0, 8); // (0,0) is invalid even with a valid CRC.
    refreshCrc();
    assertLoad(store, CampPointStatus::CORRUPT);
    useGolden();
    memset(memory.bytes + 8, 0x7f, 4); // Latitude outside the valid range.
    refreshCrc();
    assertLoad(store, CampPointStatus::CORRUPT);
}

static void test_write_failure_never_reports_save_or_delete_success()
{
    useGolden();
    memory.writeStatus = CampPointStatus::IO_ERROR;
    CampPointStore store(memory);
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.save(OTHER));
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.clear());
    assertLoad(store, CampPointStatus::OK, CAMP);
}

static void test_successful_write_requires_matching_committed_readback()
{
    useGolden();
    CampPointStore store(memory);
    memory.discardWrite = true;
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.save(OTHER));
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.clear());
    memory.discardWrite = false;
    memory.damageWrite = true;
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.save(OTHER));
    assertLoad(store, CampPointStatus::CORRUPT);
    memory.damageWrite = false;
    memory.failReadback = true;
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.save(OTHER));
    assertLoad(store, CampPointStatus::IO_ERROR);
}

static void test_unreadable_storage_is_not_overwritten()
{
    CampPointStore store(memory);
    for (const auto status : {CampPointStatus::IO_ERROR, CampPointStatus::UNAVAILABLE}) {
        memory.readStatus = status;
        TEST_ASSERT_EQUAL(status, store.save(CAMP));
        TEST_ASSERT_EQUAL(status, store.clear());
    }
    TEST_ASSERT_EQUAL(0, memory.writes);
}

static void test_explicit_save_or_clear_can_replace_corrupt_record()
{
    useGolden();
    memory.bytes[0] = 0;
    CampPointStore store(memory);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(OTHER));
    assertLoad(store, CampPointStatus::OK, OTHER);
    memory.bytes[0] = 0;
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.clear());
    assertLoad(store, CampPointStatus::EMPTY);
}

static void test_filesystem_save_replace_reload_and_delete()
{
    CampPointStore store(files);
    assertLoad(store, CampPointStatus::EMPTY);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(CAMP));
    CampPointFileStorage reopened;
    CampPointStore restarted(reopened);
    assertLoad(restarted, CampPointStatus::OK, CAMP);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.save(OTHER));
    assertLoad(store, CampPointStatus::OK, OTHER);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.clear());
    assertLoad(store, CampPointStatus::EMPTY);
    TEST_ASSERT_FALSE(FSCom.exists(TEMP_PATH));
}

static void test_filesystem_interrupted_staging_never_becomes_target()
{
    writeRaw(TEMP_PATH, GOLDEN, sizeof(GOLDEN));
    CampPointStore store(files);
    assertLoad(store, CampPointStatus::EMPTY);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(OTHER));
    writeRaw(TEMP_PATH, GOLDEN, 11);
    CampPointStore restarted(files);
    assertLoad(restarted, CampPointStatus::OK, OTHER);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, restarted.save(CAMP));
    assertLoad(restarted, CampPointStatus::OK, CAMP);
    TEST_ASSERT_FALSE(FSCom.exists(TEMP_PATH));
}

static void test_filesystem_deleted_point_is_not_restored_from_temp()
{
    CampPointStore store(files);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(CAMP));
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.clear());
    writeRaw(TEMP_PATH, GOLDEN, sizeof(GOLDEN));
    CampPointStore restarted(files);
    assertLoad(restarted, CampPointStatus::EMPTY);
    removeFile(TEMP_PATH);
}

static void test_filesystem_bad_length_or_corruption_never_uses_temp()
{
    CampPointStore store(files);
    writeRaw(TEMP_PATH, GOLDEN, sizeof(GOLDEN));
    for (size_t length = 0; length < sizeof(GOLDEN); ++length) {
        writeRaw(PATH, GOLDEN, length);
        assertLoad(store, CampPointStatus::CORRUPT);
    }
    uint8_t oversized[sizeof(GOLDEN) + 1] = {};
    memcpy(oversized, GOLDEN, sizeof(GOLDEN));
    writeRaw(PATH, oversized, sizeof(oversized));
    assertLoad(store, CampPointStatus::CORRUPT);
    oversized[9] ^= 0x01;
    writeRaw(PATH, oversized, sizeof(GOLDEN));
    assertLoad(store, CampPointStatus::CORRUPT);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(OTHER));
    assertLoad(store, CampPointStatus::OK, OTHER);
}

static void test_filesystem_blocked_staging_preserves_committed_point()
{
    CampPointStore store(files);
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(CAMP));
    TEST_ASSERT_TRUE(FSCom.mkdir(TEMP_PATH));
    writeRaw("/prefs/camp_point.dat.tmp/blocker", GOLDEN, sizeof(GOLDEN));
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.save(OTHER));
    TEST_ASSERT_EQUAL(CampPointStatus::IO_ERROR, store.clear());
    assertLoad(store, CampPointStatus::OK, CAMP);
    removeFile("/prefs/camp_point.dat.tmp/blocker");
#if defined(ARCH_PORTDUINO) && !defined(_WIN32)
    // Portduino VFSImpl::rmdir uses unlink(), which cannot remove Linux directories. Clean only
    // this fixture's now-empty directory inside the isolated filesystem mount with POSIX rmdir.
    TEST_ASSERT_NOT_NULL(portduinoVFS->mountpoint());
    const std::string stagingDirectory = std::string(portduinoVFS->mountpoint()) + TEMP_PATH;
    TEST_ASSERT_EQUAL_INT(0, ::rmdir(stagingDirectory.c_str()));
#else
    TEST_ASSERT_TRUE(FSCom.rmdir(TEMP_PATH));
#endif
    TEST_ASSERT_FALSE(FSCom.exists(TEMP_PATH));
    TEST_ASSERT_EQUAL(CampPointStatus::OK, store.save(OTHER));
    assertLoad(store, CampPointStatus::OK, OTHER);
}

extern "C" void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_empty_or_unavailable_load_clears_output);
    RUN_TEST(test_golden_record_and_signed_coordinate_round_trip);
    RUN_TEST(test_invalid_coordinates_do_not_replace_saved_point);
    RUN_TEST(test_boundary_coordinates_are_preserved);
    RUN_TEST(test_clear_survives_store_reconstruction_and_is_repeatable);
    RUN_TEST(test_every_single_byte_corruption_clears_output);
    RUN_TEST(test_unknown_version_is_reported_and_not_overwritten);
    RUN_TEST(test_valid_crc_does_not_bypass_schema_checks);
    RUN_TEST(test_write_failure_never_reports_save_or_delete_success);
    RUN_TEST(test_successful_write_requires_matching_committed_readback);
    RUN_TEST(test_unreadable_storage_is_not_overwritten);
    RUN_TEST(test_explicit_save_or_clear_can_replace_corrupt_record);
    RUN_TEST(test_filesystem_save_replace_reload_and_delete);
    RUN_TEST(test_filesystem_interrupted_staging_never_becomes_target);
    RUN_TEST(test_filesystem_deleted_point_is_not_restored_from_temp);
    RUN_TEST(test_filesystem_bad_length_or_corruption_never_uses_temp);
    RUN_TEST(test_filesystem_blocked_staging_preserves_committed_point);
    exit(UNITY_END());
}

extern "C" void loop() {}
