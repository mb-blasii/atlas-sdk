#include <atlas/core/math/math.h>
#include <cmath>

namespace atlas::core::math {

    float clamp01(float v) {
        return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }

    float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }

    bool nearlyEqual(float a, float b, float eps) {
        return std::fabs(a - b) <= eps;
    }

    bool isZero(float v, float eps) {
        return std::fabs(v) <= eps;
    }

    float degToRad(float deg) {
        return deg * DEG2RAD;
    }

    float radToDeg(float rad) {
        return rad * RAD2DEG;
    }

}