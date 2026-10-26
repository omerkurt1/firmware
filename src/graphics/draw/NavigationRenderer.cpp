#include "graphics/draw/NavigationRenderer.h"
#if HAS_SCREEN
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/draw/CompassRenderer.h"
#include "graphics/draw/UIRenderer.h"
#include "mesh/NodeDB.h"
#include "navigation/NavigationService.h"
#include <cstdio>

namespace graphics
{
namespace NavigationRenderer
{
bool supportsDisplay(OLEDDisplay *display)
{
    return display && display->getWidth() >= 128 && display->getHeight() >= 64;
}

bool selectTarget(uint32_t nodeNum)
{
    const auto *node = nodeDB ? nodeDB->getMeshNode(nodeNum) : nullptr;
    return node && !nodeInfoLiteIsIgnored(node) && navigationService && navigationService->selectTarget(nodeNum);
}

NavigationViewModel currentView()
{
    if (!navigationService) {
        NavigationViewModel unavailable;
        snprintf(unavailable.primary, sizeof(unavailable.primary), "Navigation unavailable");
        return unavailable;
    }
    // Legacy compass readings have no validated age/true-north contract. Do not fabricate a heading.
    return NavigationPresenter::present(navigationService->getSnapshot(),
                                        config.display.units == meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL);
}

void drawView(OLEDDisplay *display, const NavigationViewModel &view, const char *title, int16_t x, int16_t y)
{
    if (!supportsDisplay(display))
        return;
    const bool large = display->getHeight() >= 100 && display->getWidth() >= 200;
    const uint8_t *smallFont = large ? FONT_SMALL : FONT_SMALL_LOCAL;
    const uint8_t *distanceFont = large ? FONT_MEDIUM : FONT_MEDIUM_LOCAL;
    const int width = display->getWidth() - 4;
    const int height = display->getHeight();
    auto line = [&](int top, const char *text, int maxWidth) {
        const auto clipped = UIRenderer::truncateStringWithEmotes(display, text, maxWidth);
        display->drawString(x + 2, y + top, clipped.c_str());
    };
    display->setColor(WHITE);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(smallFont);
    line(0, title, width);
    display->setFont(view.showDistance ? distanceFont : smallFont);
    line(height * 14 / 64, view.primary, view.showArrow ? width - 32 : width);
    if (view.showArrow)
        CompassRenderer::drawArrowToNode(display, x + width - 14, y + height * 24 / 64, 24, view.arrowDegrees);
    display->setFont(smallFont);
    line(height * 35 / 64, view.detail, width);
    line(height * 49 / 64, view.ages, width);
}

void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    (void)state;
    const auto view = currentView();
    const uint32_t target = view.targetNode;
    char title[48] = "Find a device";
    if (view.targetIsCamp) {
        snprintf(title, sizeof(title), "Return to camp");
    } else if (target) {
        const auto *node = nodeDB ? nodeDB->getMeshNode(target) : nullptr;
        if (node && nodeInfoLiteHasUser(node) && node->long_name[0])
            snprintf(title, sizeof(title), "Find: %s", sanitizeString(node->long_name).c_str());
        else
            snprintf(title, sizeof(title), "Find: !%08x", static_cast<unsigned int>(target));
    }
    drawView(display, view, title, x, y);
}
} // namespace NavigationRenderer
} // namespace graphics
#endif
