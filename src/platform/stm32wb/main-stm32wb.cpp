#include "configuration.h"
#include "AnimalRadio.h"
#include "main.h"

#include <cstring>

void earlyInitVariant()
{
    pinMode(WB_CC1101_CS, OUTPUT);
    digitalWrite(WB_CC1101_CS, HIGH);
    pinMode(WB_SX1262_CS, OUTPUT);
    digitalWrite(WB_SX1262_CS, HIGH);
}

void lateInitVariant()
{
    wbAnimalRadio::setup();
}

void setBluetoothEnable(bool) {}

void updateBatteryLevel(uint8_t) {}

void playStartMelody() {}

void playShutdownMelody() {}

void playGPSEnableBeep() {}

void playGPSDisableBeep() {}

void cpuDeepSleep(uint32_t)
{
    // The ST-LINK bench build stays available for serial diagnostics.
}

bool getDeviceId(uint8_t *deviceId)
{
    const uint32_t uid[] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    memcpy(deviceId, uid, sizeof(uid));
    return true;
}

void getMacAddr(uint8_t *address)
{
    uint8_t uid[12];
    getDeviceId(uid);
    uint64_t hash = 14695981039346656037ULL;
    for (const auto byte : uid) {
        hash = (hash ^ byte) * 1099511628211ULL;
    }
    for (size_t i = 0; i < 6; i++) {
        address[i] = static_cast<uint8_t>(hash >> (8 * i));
    }
    address[0] = (address[0] & 0xfe) | 0x02;
}
