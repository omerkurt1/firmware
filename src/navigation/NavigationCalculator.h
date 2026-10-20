#pragma once

#include "navigation/NavigationTypes.h"

class NavigationCalculator
{
  public:
    static bool isCoordinateValid(const NavigationCoordinate &coordinate);
    static float normalizeDegrees360(float degrees);
    static float normalizeDeltaDegrees180(float degrees);
    static NavigationGeometry calculate(const NavigationCoordinate &origin, const NavigationCoordinate &target,
                                        const NavigationHeading &heading = {});
};
