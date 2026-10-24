// NavigationGnssRmcReader in src/navigation/NavigationGnssRmcReader.cpp captures course and speed
// from one bounded, checksum-checked RMC. Regression coverage prevents mixed epochs, stale optional
// fields, parser overflow and receipt time being promoted into trusted measurement time.
#include "MeshTypes.h"
#include "TestUtil.h"
#include "navigation/NavigationGnssRmcReader.h"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <unity.h>

namespace
{
constexpr uint64_t NOW = 10000;
NavigationGnssRmcReader reader;

std::string rmc(const char *speed = "4.0", const char *course = "90.0", const char *status = "A", const char *suffix = ",A,V",
                const char *talker = "GN")
{
    return std::string(talker) + "RMC,120000.00," + status + ",4100.0000,N,02900.0000,E," + speed + "," + course + ",231026,," +
           suffix;
}

std::string frame(const std::string &body)
{
    uint8_t checksum = 0;
    for (const char c : body)
        checksum ^= static_cast<uint8_t>(c);
    char trailer[8];
    snprintf(trailer, sizeof(trailer), "*%02X\r\n", checksum);
    return "$" + body + trailer;
}

void feed(const std::string &bytes, uint64_t now = NOW)
{
    for (const char c : bytes)
        reader.encode(c, now);
}

void assertEmpty()
{
    const auto observation = reader.copyObservation();
    TEST_ASSERT_FALSE(observation.received);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::NONE, observation.heading.source);
    TEST_ASSERT_FALSE(observation.heading.measurementValid);
    TEST_ASSERT_FALSE(observation.heading.speedValid);
}
} // namespace

void setUp()
{
    reader.reset();
}
void tearDown() {}

static void test_rmc_coherent_course_and_speed_have_no_measurement_time()
{
    feed(frame(rmc()));
    const auto observation = reader.copyObservation();
    TEST_ASSERT_TRUE(observation.received);
    TEST_ASSERT_EQUAL_UINT64(NOW, observation.receivedAtUptimeMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90, observation.heading.degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.05778f, observation.heading.speedMetersPerSecond);
    TEST_ASSERT_EQUAL(NavigationHeadingSource::MOVEMENT_COURSE, observation.heading.source);
    TEST_ASSERT_EQUAL(NavigationHeadingReference::TRUE_NORTH, observation.heading.reference);
    TEST_ASSERT_TRUE(observation.heading.measurementValid);
    TEST_ASSERT_TRUE(observation.heading.fixValid);
    TEST_ASSERT_TRUE(observation.heading.speedValid);
    TEST_ASSERT_FALSE(observation.heading.timeKnown);
    TEST_ASSERT_FALSE(observation.heading.speedTimeKnown);
    TEST_ASSERT_EQUAL_UINT64(0, observation.heading.measuredAtUptimeMs);
    TEST_ASSERT_EQUAL_UINT64(0, observation.heading.speedMeasuredAtUptimeMs);
    const auto result = NavigationHeadingValidator::evaluate(observation.heading, NOW);
    TEST_ASSERT_EQUAL(NavigationHeadingState::TIME_UNKNOWN, result.state);
    TEST_ASSERT_FALSE(result.heading.valid);
}

static void test_rmc_known_checksum_fixture_and_talkers()
{
    feed("$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57\r\n");
    assertEmpty(); // The published example's checksum is wrong; the payload XOR is 0x2D.
    feed("$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*2D\r\n");
    TEST_ASSERT_TRUE(reader.copyObservation().received);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 77.52f, reader.copyObservation().heading.degrees);
    for (const auto talker : {"GP", "GN", "GA", "GL", "GB", "BD"}) {
        reader.reset();
        feed(frame(rmc("4", "90", "A", ",A", talker)));
        TEST_ASSERT_TRUE(reader.copyObservation().received);
    }
}

static void test_rmc_legacy_and_supported_fix_modes()
{
    for (const auto suffix : {"", ",A", ",D", ",F", ",R", ",A,V", ",A,S"}) {
        reader.reset();
        feed(frame(rmc("4", "90", "A", suffix)));
        TEST_ASSERT_TRUE(reader.copyObservation().received);
    }
}

static void test_rmc_invalid_fix_replaces_previous_observation()
{
    for (const auto status : {"V", "", "AA", "X"}) {
        feed(frame(rmc()));
        feed(frame(rmc("4", "90", status)));
        assertEmpty();
    }
}

static void test_rmc_unusable_mode_and_integrity_clear_previous_observation()
{
    for (const auto suffix : {",E", ",M", ",S", ",N", ",", ",AA", ",A,C", ",A,U", ",A,", ",A,V,extra"}) {
        feed(frame(rmc()));
        feed(frame(rmc("4", "90", "A", suffix)));
        assertEmpty();
    }
}

static void test_rmc_missing_fields_never_mix_with_old_values()
{
    feed(frame(rmc()));
    feed(frame(rmc("", "180")));
    assertEmpty();
    feed(frame(rmc("8", "")));
    assertEmpty();
    feed(frame(rmc("8", "180")), NOW + 100);
    const auto observation = reader.copyObservation();
    TEST_ASSERT_TRUE(observation.received);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180, observation.heading.degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.11556f, observation.heading.speedMetersPerSecond);
    TEST_ASSERT_EQUAL_UINT64(NOW + 100, observation.receivedAtUptimeMs);
}

static void test_rmc_malformed_numeric_fields_are_rejected()
{
    for (const auto bad :
         {"-1", "+1", "nan", "inf", "1e2", " 1", "1x", ".", ".5", "1.", "1.2.3", "99999999999999999999999999999999999999999"}) {
        feed(frame(rmc()));
        feed(frame(rmc(bad)));
        assertEmpty();
        feed(frame(rmc("4", bad)));
        assertEmpty();
    }
}

static void test_rmc_angle_bounds_and_stopped_speed()
{
    feed(frame(rmc("0", "0")));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
    TEST_ASSERT_EQUAL_FLOAT(0, reader.copyObservation().heading.speedMetersPerSecond);
    feed(frame(rmc("4", "359.999")));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
    for (const auto bad : {"360", "360.001", "999"}) {
        feed(frame(rmc("4", bad)));
        assertEmpty();
    }
}

static void test_rmc_decimal_precision_and_leading_zeros()
{
    feed(frame(rmc("0001.234567", "00359.123456")));
    auto observation = reader.copyObservation();
    TEST_ASSERT_TRUE(observation.received);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 359.123456f, observation.heading.degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.6351161f, observation.heading.speedMetersPerSecond);
    feed(frame(rmc("0.000001", "0.000001")));
    observation = reader.copyObservation();
    TEST_ASSERT_TRUE(observation.received);
    TEST_ASSERT_FLOAT_WITHIN(0.000000001f, 0.000001f, observation.heading.degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.000000001f, 0.0000005144444f, observation.heading.speedMetersPerSecond);
}

static void test_rmc_checksum_damage_and_recovery()
{
    auto damaged = frame(rmc());
    damaged[damaged.find('*') + 1] = 'Z';
    feed(frame(rmc()));
    feed(damaged);
    assertEmpty();
    damaged = frame(rmc());
    damaged[1] = 'P';
    feed(damaged);
    assertEmpty();
    feed(frame(rmc()));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
}

static void test_rmc_missing_or_extra_checksum_characters()
{
    for (const auto trailer : {"\r\n", "*1\n", "*000\n", "*00junk\n", "*GG\n"}) {
        feed(frame(rmc()));
        feed("$" + rmc() + trailer);
        assertEmpty();
    }
}

static void test_stream_partial_sentence_is_not_published()
{
    const auto bytes = frame(rmc());
    for (size_t i = 0; i < bytes.size() - 2; ++i) {
        reader.encode(bytes[i], NOW);
        assertEmpty();
    }
    reader.encode('\r', NOW + 50);
    TEST_ASSERT_TRUE(reader.copyObservation().received);
    TEST_ASSERT_EQUAL_UINT64(NOW + 50, reader.copyObservation().receivedAtUptimeMs);
    reader.encode('\n', NOW + 100);
    TEST_ASSERT_EQUAL_UINT64(NOW + 50, reader.copyObservation().receivedAtUptimeMs);
}

static void test_stream_other_sentences_do_not_refresh_observation()
{
    feed(frame(rmc()));
    feed(frame("GPGGA,120001,4100,N,02900,E,1,12,1,0,M,0,M,,"), NOW + 5000);
    feed(frame("GPVTG,180,T,,M,8,N,14,K,A"), NOW + 6000);
    TEST_ASSERT_EQUAL_UINT64(NOW, reader.copyObservation().receivedAtUptimeMs);
    TEST_ASSERT_EQUAL_FLOAT(90, reader.copyObservation().heading.degrees);
}

static void test_stream_reset_clears_observation_and_partial_sentence()
{
    feed(frame(rmc()));
    feed("$GNRMC,120000");
    reader.reset();
    feed(frame(rmc()).substr(12));
    assertEmpty();
    feed(frame(rmc()));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
}

static void test_stream_overflow_noise_and_new_start_recover()
{
    feed(frame(rmc()));
    feed("$" + std::string(200, 'X'));
    assertEmpty();
    feed("noise$GNRMC,partial");
    feed(frame(rmc()));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
    feed(std::string("$bad") + '\0');
    assertEmpty();
    feed(frame(rmc()));
    TEST_ASSERT_TRUE(reader.copyObservation().received);
}

static void test_stream_lf_and_lowercase_checksum_are_supported()
{
    auto bytes = frame(rmc());
    bytes.erase(bytes.size() - 2, 1);
    for (size_t i = bytes.find('*') + 1; i < bytes.size() - 1; ++i)
        if (bytes[i] >= 'A' && bytes[i] <= 'F')
            bytes[i] += 'a' - 'A';
    feed(bytes);
    TEST_ASSERT_TRUE(reader.copyObservation().received);
}

static void test_receipt_time_zero_and_extended_clock_are_not_measurement_time()
{
    for (const uint64_t now : {uint64_t{0}, uint64_t{UINT32_MAX} + 1000}) {
        feed(frame(rmc()), now);
        const auto observation = reader.copyObservation();
        TEST_ASSERT_EQUAL_UINT64(now, observation.receivedAtUptimeMs);
        TEST_ASSERT_EQUAL(NavigationHeadingState::TIME_UNKNOWN,
                          NavigationHeadingValidator::evaluate(observation.heading, now).state);
    }
}

static void test_rmc_replay_never_acquires_trusted_time()
{
    feed(frame(rmc()));
    feed(frame(rmc()), NOW + 100000);
    const auto sample = reader.copyObservation().heading;
    TEST_ASSERT_EQUAL(NavigationHeadingState::TIME_UNKNOWN, NavigationHeadingValidator::evaluate(sample, NOW + 100000).state);
}

static void test_rmc_truncated_structure_is_rejected()
{
    feed(frame(rmc()));
    feed(frame("GNRMC,120000,A,4100,N,02900,E,4,90"));
    assertEmpty();
    feed(frame(rmc("4", "90", "A", ",A", "1N")));
    assertEmpty();
}

extern "C" void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_rmc_coherent_course_and_speed_have_no_measurement_time);
    RUN_TEST(test_rmc_known_checksum_fixture_and_talkers);
    RUN_TEST(test_rmc_legacy_and_supported_fix_modes);
    RUN_TEST(test_rmc_invalid_fix_replaces_previous_observation);
    RUN_TEST(test_rmc_unusable_mode_and_integrity_clear_previous_observation);
    RUN_TEST(test_rmc_missing_fields_never_mix_with_old_values);
    RUN_TEST(test_rmc_malformed_numeric_fields_are_rejected);
    RUN_TEST(test_rmc_angle_bounds_and_stopped_speed);
    RUN_TEST(test_rmc_decimal_precision_and_leading_zeros);
    RUN_TEST(test_rmc_checksum_damage_and_recovery);
    RUN_TEST(test_rmc_missing_or_extra_checksum_characters);
    RUN_TEST(test_stream_partial_sentence_is_not_published);
    RUN_TEST(test_stream_other_sentences_do_not_refresh_observation);
    RUN_TEST(test_stream_reset_clears_observation_and_partial_sentence);
    RUN_TEST(test_stream_overflow_noise_and_new_start_recover);
    RUN_TEST(test_stream_lf_and_lowercase_checksum_are_supported);
    RUN_TEST(test_receipt_time_zero_and_extended_clock_are_not_measurement_time);
    RUN_TEST(test_rmc_replay_never_acquires_trusted_time);
    RUN_TEST(test_rmc_truncated_structure_is_rejected);
    exit(UNITY_END());
}

extern "C" void loop() {}
