#pragma once

#include <cstddef>
#include <cstdint>

class OLEDDisplay;
struct OLEDDisplayUiState;

namespace wbAnimalRadio
{
void setup();
void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *, int16_t x, int16_t y);
} // namespace wbAnimalRadio
