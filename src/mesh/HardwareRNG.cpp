#include "HardwareRNG.h"

#include <algorithm>
#include <cstring>
#include <random>

#include "configuration.h"

#if HAS_RADIO
#include "RadioLibInterface.h"
#endif

#if defined(ARCH_NRF54L)
#include <nRF54Crypto.h>
#elif defined(ARCH_NRF52)
#include <Adafruit_nRFCrypto.h>
extern Adafruit_nRFCrypto nRFCrypto;
#elif defined(ARCH_ESP32)
#include <esp_system.h>
#elif defined(ARCH_RP2040)
#include <Arduino.h>
#elif defined(ARCH_STM32WB)
#include <stm32wbxx_hal_rng.h>
#elif defined(ARCH_PORTDUINO)
#include <random>
#include <unistd.h>
#ifdef __linux__
#include <sys/random.h> // getrandom()
#elif defined(_WIN32)
// Order is load-bearing, hence the blank line: bcrypt.h uses LONG/ULONG from
// windows.h and does not include it itself.
#include <windows.h>

#include <bcrypt.h> // BCryptGenRandom()
#else
#include <stdlib.h> // arc4random_buf() on Darwin/BSD
#endif
#endif

namespace HardwareRNG
{

namespace
{
#ifndef ARCH_STM32WB
void fillWithRandomDevice(uint8_t *buffer, size_t length)
{
    std::random_device rd;
    size_t offset = 0;
    while (offset < length) {
        uint32_t value = rd();
        size_t toCopy = std::min(length - offset, sizeof(value));
        memcpy(buffer + offset, &value, toCopy);
        offset += toCopy;
    }
}
#else
bool fillWithStm32wbRng(uint8_t *buffer, size_t length)
{
    RCC_OscInitTypeDef oscillator{};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    oscillator.HSI48State = RCC_HSI48_ON;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        return false;
    }
    __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
    __HAL_RCC_RNG_CLK_ENABLE();

    RNG_HandleTypeDef rng{};
    rng.Instance = RNG;
    if (HAL_RNG_Init(&rng) != HAL_OK) {
        return false;
    }
    size_t offset = 0;
    while (offset < length) {
        uint32_t value;
        if (HAL_RNG_GenerateRandomNumber(&rng, &value) != HAL_OK) {
            break;
        }
        const size_t chunk = std::min(length - offset, sizeof(value));
        memcpy(buffer + offset, &value, chunk);
        offset += chunk;
    }
    HAL_RNG_DeInit(&rng);
    if (offset != length) {
        memset(buffer, 0, length);
        return false;
    }
    return true;
}
#endif

#if HAS_RADIO
bool mixWithLoRaEntropy(uint8_t *buffer, size_t length)
{
    // Only attempt to pull entropy from the modem if it is initialized and exposes the helper.
    // When the radio stack is disabled or has not yet been configured, we simply skip this step
    // and return false so callers know no extra mixing occurred.
    RadioLibInterface *radio = RadioLibInterface::instance;
    if (!radio) {
        // This path can run during portduinoSetup() before the console is initialized,
        // both for unit-test binaries and the simulator's meshtasticd; LOG_* dereferences `console`.
        if (console) {
            LOG_ERROR("No radio instance available to provide entropy");
        }
        return false;
    }

    constexpr size_t chunkSize = 16;
    uint8_t scratch[chunkSize];
    size_t offset = 0;
    bool mixed = false;

    while (offset < length) {
        size_t toCopy = std::min(length - offset, chunkSize);

        // randomBytes() returns false if the modem does not support it or is not ready
        // (for instance, when the radio is powered down). We break immediately to avoid
        // blocking or returning partially-filled entropy and simply report failure.
        if (!radio->randomBytes(scratch, toCopy)) {
            break;
        }

        for (size_t i = 0; i < toCopy; ++i) {
            buffer[offset + i] ^= scratch[i];
        }

        mixed = true;
        offset += toCopy;
    }

    // Avoid leaving the modem-sourced bytes sitting on the stack longer than needed.
    if (mixed) {
        memset(scratch, 0, sizeof(scratch));
    }

    return mixed;
}
#endif
} // namespace

bool fill(uint8_t *buffer, size_t length, bool useRadioEntropy)
{
    if (!buffer || length == 0) {
        return false;
    }

    bool filled = false;

#if defined(ARCH_NRF54L)
    // CRACEN TRNG
    nRF54Crypto.begin();
    filled = nRF54Crypto.random(buffer, length);
    nRF54Crypto.end();
#elif defined(ARCH_NRF52)
    // The Nordic SDK RNG provides cryptographic-quality randomness backed by hardware.
    nRFCrypto.begin();
    auto result = nRFCrypto.Random.generate(buffer, length);
    nRFCrypto.end();
    filled = result;
#elif defined(ARCH_ESP32)
    // ESP32 exposes a true RNG via esp_fill_random().
    esp_fill_random(buffer, length);
    filled = true;
#elif defined(ARCH_RP2040)
    // RP2040 has a hardware random number generator accessible through the Arduino core.
    size_t offset = 0;
    while (offset < length) {
        uint32_t value = rp2040.hwrand32();
        size_t toCopy = std::min(length - offset, sizeof(value));
        memcpy(buffer + offset, &value, toCopy);
        offset += toCopy;
    }
    filled = true;
#elif defined(ARCH_PORTDUINO)
    // Prefer the host OS RNG first when running under Portduino.
#ifdef __linux__
    ssize_t generated = ::getrandom(buffer, length, 0);
    if (generated == static_cast<ssize_t>(length)) {
        filled = true;
    }
#elif defined(_WIN32)
    // No getrandom/arc4random on Windows; BCryptGenRandom is the documented CSPRNG.
    // Preferred over std::random_device, whose libstdc++ Windows backend reports entropy() == 0.
    if (BCryptGenRandom(NULL, buffer, static_cast<ULONG>(length), BCRYPT_USE_SYSTEM_PREFERRED_RNG) == 0) { // STATUS_SUCCESS
        filled = true;
    }
#elif defined(__EMSCRIPTEN__)
    // Browser/wasm: no getrandom/arc4random - fall through to std::random_device,
    // which emscripten backs with crypto.getRandomValues().
#else
    // arc4random_buf is available on Darwin/BSD and cannot fail.
    ::arc4random_buf(buffer, length);
    filled = true;
#endif

    if (!filled) {
        fillWithRandomDevice(buffer, length);
        filled = true;
    }
#endif

#ifdef ARCH_STM32WB
    // Radio entropy is mixed only into a buffer already filled by a working RNG.
    filled = fillWithStm32wbRng(buffer, length);
    if (!filled) {
        return false;
    }
#else
    if (!filled) {
        // As a last resort, fall back to std::random_device. This should only be reached
        // if a platform-specific source was unavailable.
        fillWithRandomDevice(buffer, length);
        filled = true;
    }
#endif

#if HAS_RADIO
    if (useRadioEntropy) {
        // Best-effort: if the radio is active and can provide modem entropy, XOR it over the
        // buffer to improve overall quality. We consider the filling a success if either a
        // good platform RNG or the modem RNG provided data, so we return true as long as at
        // least one of those steps succeeded.
        filled = mixWithLoRaEntropy(buffer, length) || filled;
    }
#endif

    return filled;
}

bool seed(uint32_t &seedOut)
{
    uint32_t candidate = 0;
    if (!fill(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate), true)) {
        return false;
    }
    seedOut = candidate;
    return true;
}

} // namespace HardwareRNG
