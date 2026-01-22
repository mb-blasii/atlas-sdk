#pragma once
#include <atlas/core/vectors/vec2.h>

namespace atlas::core::shape {

    struct Rect {
        vec::Vec2 center;
        vec::Vec2 halfExtents;
    };

    struct Circle {
        vec::Vec2 center;
        float radius;
    };

    struct Capsule2D {
        vec::Vec2 a;
        vec::Vec2 b;
        float radius;
    };

#pragma region utility functions

    float distancePointSegmentSq(const vec::Vec2 &p, const vec::Vec2 &a, const vec::Vec2 &b);

    vec::Vec2 clampPointRect(const vec::Vec2 &p, const Rect &r);

#pragma endregion

#pragma region overlap

    // Rect-Rect
    bool overlap(const Rect &r1, const Rect &r2);

    // Circle-Circle
    bool overlap(const Circle &c1, const Circle &c2);

    // Capsule-Capsule
    bool overlap(const Capsule2D &a, const Capsule2D &b);

    // Rect-Circle
    bool overlap(const Rect &r, const Circle &c);
    bool overlap(const Circle &c, const Rect &r);

    // Capsule-Circle
    bool overlap(const Capsule2D &cap, const Circle &c);
    bool overlap(const Circle &c, const Capsule2D &cap);

    // Capsule-Rect
    bool overlap(const Capsule2D &cap, const Rect &r);
    bool overlap(const Rect &r, const Capsule2D &cap);

#pragma endregion

}
