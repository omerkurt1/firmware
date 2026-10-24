#include "navigation/NavigationGnssRmcReader.h"

#include <cmath>
#include <cstring>

namespace
{
int hexDigit(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

bool decimal(const char *text, float &value)
{
    if (*text < '0' || *text > '9')
        return false;
    value = 0;
    float fractionalScale = 1;
    bool point = false;
    for (const char *p = text; *p; ++p) {
        if (*p == '.' && !point && p[1] >= '0' && p[1] <= '9') {
            point = true;
        } else if (*p < '0' || *p > '9') {
            return false;
        } else if (point) {
            fractionalScale *= 0.1f;
            value += (*p - '0') * fractionalScale;
        } else {
            value = value * 10 + (*p - '0');
        }
    }
    return std::isfinite(value);
}

bool singleChar(const char *text, char expected)
{
    return text[0] == expected && text[1] == '\0';
}
} // namespace

void NavigationGnssRmcReader::reset()
{
    length = 0;
    collecting = false;
    observation = {};
}

void NavigationGnssRmcReader::encode(char byte, uint64_t receivedAtUptimeMs)
{
    if (byte == '$') {
        if (collecting)
            observation = {};
        length = 0;
        collecting = true;
        return;
    }
    if (!collecting)
        return;
    if (byte == '\r' || byte == '\n') {
        buffer[length] = '\0';
        finish(receivedAtUptimeMs);
        collecting = false;
        length = 0;
        return;
    }
    if (byte < 32 || byte > 126 || length == sizeof(buffer) - 1) {
        reset();
        return;
    }
    buffer[length++] = byte;
}

void NavigationGnssRmcReader::finish(uint64_t receivedAtUptimeMs)
{
    char *checksum = strchr(buffer, '*');
    if (!checksum || strlen(checksum) != 3 || hexDigit(checksum[1]) < 0 || hexDigit(checksum[2]) < 0) {
        observation = {};
        return;
    }
    uint8_t parity = 0;
    for (const char *p = buffer; p != checksum; ++p)
        parity ^= static_cast<uint8_t>(*p);
    if (parity != ((hexDigit(checksum[1]) << 4) | hexDigit(checksum[2]))) {
        observation = {};
        return;
    }
    *checksum = '\0';
    if (strlen(buffer) < 6 || strncmp(buffer + 2, "RMC,", 4) != 0)
        return;

    // Never retain a field from an earlier RMC, including a checksum-valid no-fix sentence.
    observation = {};
    if (buffer[0] < 'A' || buffer[0] > 'Z' || buffer[1] < 'A' || buffer[1] > 'Z')
        return;
    char *fields[14] = {buffer};
    size_t count = 1;
    for (char *p = buffer; *p; ++p) {
        if (*p != ',')
            continue;
        if (count == 14)
            return;
        *p = '\0';
        fields[count++] = p + 1;
    }
    if (count < 12 || !singleChar(fields[2], 'A'))
        return;
    if (count >= 13 && !(singleChar(fields[12], 'A') || singleChar(fields[12], 'D') || singleChar(fields[12], 'F') ||
                         singleChar(fields[12], 'R')))
        return;
    // V means integrity information unavailable on supported u-blox receivers, not RMC status V.
    if (count == 14 && !(singleChar(fields[13], 'S') || singleChar(fields[13], 'V')))
        return;
    float knots = 0;
    float degrees = 0;
    if (!decimal(fields[7], knots) || !decimal(fields[8], degrees) || degrees >= 360)
        return;

    observation.heading.source = NavigationHeadingSource::MOVEMENT_COURSE;
    observation.heading.reference = NavigationHeadingReference::TRUE_NORTH;
    observation.heading.degrees = degrees;
    observation.heading.speedMetersPerSecond = knots * (1852.0f / 3600.0f);
    observation.heading.measurementValid = true;
    observation.heading.fixValid = true;
    observation.heading.speedValid = true;
    // Leave both measurement timestamps unknown: buffering, UTC mapping and replay are unverified.
    observation.receivedAtUptimeMs = receivedAtUptimeMs;
    observation.received = true;
}
