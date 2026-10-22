#pragma once

#include "configuration.h"
#if HAS_SCREEN
#include "navigation/NavigationViewModel.h"
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

namespace graphics
{
namespace NavigationRenderer
{
bool supportsDisplay(OLEDDisplay *display);
bool selectTarget(uint32_t nodeNum);
NavigationViewModel currentView();
void drawView(OLEDDisplay *display, const NavigationViewModel &view, const char *title, int16_t x = 0, int16_t y = 0);
void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
} // namespace NavigationRenderer
} // namespace graphics
#endif
