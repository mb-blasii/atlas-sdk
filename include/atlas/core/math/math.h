#pragma once
#include "constants.h"

namespace atlas::core::math {

    float clamp01(float v);

    float lerp(float a, float b, float t);

    bool nearlyEqual(float a, float b, float eps = EPS);
    bool isZero(float v, float eps = EPS);

    float degToRad(float deg);
    float radToDeg(float rad);

}
