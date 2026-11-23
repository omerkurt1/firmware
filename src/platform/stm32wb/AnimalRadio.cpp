#include "AnimalRadio.h"
#include "configuration.h"
#include "concurrency/OSThread.h"
#include "graphics/ScreenFonts.h"
#include "main.h"
#include "tags/Ht12eDecoder.h"
#include <OLEDDisplay.h>
#include <RadioLib.h>
#include <SPI.h>
#include <array>
#include <cstdio>

namespace wbAnimalRadio
{
namespace
{
struct Edge {
    uint32_t atUs;
    bool level;
};

struct Tag {
    uint32_t seenMs;
    uint16_t id;
    int16_t rssi;
    bool hasRssi;
};

std::array<Edge, 256> edges;
volatile uint8_t edgeRead = 0;
volatile uint8_t edgeWrite = 0;
volatile bool overflow = false;
std::array<Tag, 200> tags;
size_t tagCount = 0;
size_t latestTag = 0;
bool ready = false;

void onEdge()
{
    const auto index = edgeWrite;
    const auto next = static_cast<uint8_t>(index + 1);
    if (next == edgeRead) {
        overflow = true;
        return;
    }
    edges[index] = {micros(), digitalRead(WB_CC1101_GDO0) == HIGH};
    __DMB();
    edgeWrite = next;
}

void recordTag(uint16_t id, int16_t rssi, bool hasRssi)
{
    const uint32_t now = millis();
    size_t index = 0;
    size_t oldest = 0;
    while (index < tagCount && tags[index].id != id) {
        if (now - tags[index].seenMs > now - tags[oldest].seenMs) {
            oldest = index;
        }
        ++index;
    }
    if (index == tags.size()) {
        index = oldest;
    } else if (index == tagCount) {
        ++tagCount;
    }
    tags[index] = {now, id, rssi, hasRssi};
    latestTag = index;
}

class Receiver final : public concurrency::OSThread
{
  public:
    Receiver() : OSThread("WB tags", 2) {}

    int16_t begin()
    {
        ConfigFSK_t settings;
        settings.frequency = 433.92f;
        settings.bitRate = 3.0f;
        settings.receiverBandwidth = 135.0f;
        auto result = radio.begin(settings);
        if (result == RADIOLIB_ERR_NONE) {
            result = radio.setOOK(true);
        }
        if (result == RADIOLIB_ERR_NONE) {
            result = radio.receiveDirectAsync();
        }
        if (result == RADIOLIB_ERR_NONE) {
            pinMode(WB_CC1101_GDO0, INPUT);
            attachInterrupt(digitalPinToInterrupt(WB_CC1101_GDO0), onEdge, CHANGE);
        }
        return result;
    }

  protected:
    int32_t runOnce() override
    {
        // Copy one edge with interrupts held briefly, then decode in the main loop.
        for (unsigned i = 0; i < edges.size(); ++i) {
            const auto interruptState = __get_PRIMASK();
            __disable_irq();
            const bool droppedEdges = overflow;
            if (droppedEdges) {
                edgeRead = edgeWrite;
                overflow = false;
            }
            const bool available = edgeRead != edgeWrite;
            Edge edge{};
            if (available) {
                edge = edges[edgeRead];
                ++edgeRead;
            }
            __set_PRIMASK(interruptState);
            if (droppedEdges) {
                havePrevious = false;
                haveRssi = false;
                decoder.reset();
            }
            if (!available) {
                break;
            }
            uint16_t id;
            if (havePrevious && edge.level == previous.level) {
                decoder.reset();
            } else if (havePrevious && decoder.pulse(previous.level, edge.atUs - previous.atUs, id)) {
                // Only associate a sample taken during the final high pulse of this verified word.
                recordTag(id, lastRssi, haveRssi && sampledHighAtUs == previous.atUs);
            }
            previous = edge;
            havePrevious = true;
        }
        if (havePrevious && previous.level && previous.atUs != sampledHighAtUs && edgeRead == edgeWrite &&
            digitalRead(WB_CC1101_GDO0) == HIGH) {
            const auto observedWrite = edgeWrite;
            const auto sample = static_cast<int16_t>(radio.getRSSI());
            if (edgeWrite == observedWrite && edgeRead == observedWrite && !overflow &&
                digitalRead(WB_CC1101_GDO0) == HIGH) {
                lastRssi = sample;
                sampledHighAtUs = previous.atUs;
                haveRssi = true;
            }
        }
        return RUN_SAME;
    }

  private:
    Module module{WB_CC1101_CS, WB_CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC, SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0)};
    CC1101 radio{&module};
    Ht12eDecoder decoder;
    Edge previous{};
    uint32_t sampledHighAtUs = 0;
    int16_t lastRssi = 0;
    bool havePrevious = false;
    bool haveRssi = false;
};
} // namespace

void setup()
{
    SPI.begin();
    auto *receiver = new Receiver;
    const auto result = receiver->begin();
    ready = result == RADIOLIB_ERR_NONE;
    if (!ready) {
        LOG_WARN("CC1101 unavailable: %d", result);
        delete receiver;
    } else {
        LOG_INFO("CC1101 receiving 433.92 MHz OOK");
    }
}

void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setColor(WHITE);
    display->setFont(FONT_SMALL_LOCAL);
    display->drawString(x + 2, y, "Animal tags");
    if (!tagCount) {
        display->drawString(x + 2, y + 18, ready ? "Waiting for tag" : "433 MHz unavailable");
        return;
    }
    const auto &tag = tags[latestTag];
    char line[28];
    snprintf(line, sizeof(line), "Last tag %03X (%u seen)", static_cast<unsigned>(tag.id), static_cast<unsigned>(tagCount));
    display->drawString(x + 2, y + 14, line);
    snprintf(line, sizeof(line), "Heard %lu s ago", static_cast<unsigned long>((millis() - tag.seenMs) / 1000));
    display->drawString(x + 2, y + 29, line);
    if (tag.hasRssi) {
        snprintf(line, sizeof(line), "Signal %d dBm", tag.rssi);
        display->drawString(x + 2, y + 44, line);
    }
}
} // namespace wbAnimalRadio
