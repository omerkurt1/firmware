#pragma once

#include "navigation/NavigationTypes.h"

struct NavigationViewModel {
    uint32_t targetNode = 0;
    bool targetIsCamp = false;
    char primary[32] = {};
    char detail[48] = {};
    char ages[40] = {};
    bool showDistance = false;
    bool showArrow = false;
    float arrowDegrees = 0;
};

class NavigationPresenter
{
  public:
    static NavigationViewModel present(const NavigationSnapshot &snapshot, bool imperial = false);
    static const char *campFeedback(CampActionStatus status);
};
