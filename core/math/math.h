#pragma once
#include <cmath>
#include "constants.h"

namespace atlas::core::math {

    inline float clamp01(float v) {
        return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }

    inline float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }

    inline bool nearlyEqual(float a, float b, float eps = EPS) {
        return std::fabs(a - b) <= eps;
    }

    inline bool isZero(float v, float eps = EPS) {
        return std::fabs(v) <= eps;
    }

    inline float degToRad(float deg) {
        return deg * DEG2RAD;
    }

    inline float radToDeg(float rad) {
        return rad * RAD2DEG;
    }

}
