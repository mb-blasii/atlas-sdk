#pragma once
#include <atlas/core/vectors/vec2.h>

namespace atlas::physics::shape {

    struct Shape2D {
        void* ctx = nullptr; //context: might be used in future to bind a shape with an owning object
    };

    struct Rect : Shape2D{
        core::vec::Vec2 center;
        core::vec::Vec2 halfExtents;

        Rect() : Shape2D(nullptr) {}
        Rect(const core::vec::Vec2& c, const core::vec::Vec2& he) : Shape2D(nullptr), center(c), halfExtents(he) {}
    };

    struct Circle : Shape2D{
        core::vec::Vec2 center;
        float radius = 1.0f;

        Circle() : Shape2D(nullptr) {}
        Circle(const core::vec::Vec2& c, float r) : Shape2D(nullptr), center(c), radius(r) {}
    };

    struct Capsule2D : Shape2D {
        core::vec::Vec2 a;
        core::vec::Vec2 b;
        float radius = 1.0f;

        Capsule2D() : Shape2D(nullptr) {}
        Capsule2D(const core::vec::Vec2& a, const core::vec::Vec2& b, float r) : Shape2D(nullptr), a(a), b(b), radius(r) {}
    };

#pragma region utility functions

    float distancePointSegmentSq(const core::vec::Vec2 &p, const core::vec::Vec2 &a, const core::vec::Vec2 &b);

    core::vec::Vec2 clampPointRect(const core::vec::Vec2 &p, const Rect &r);

#pragma endregion

#pragma region overlap

    // Point-Circle
    bool overlap(const core::vec::Vec2& point, const Circle& c);

    // Point-Rect
    bool overlap(const core::vec::Vec2& point, const Rect& r);

    // Point-Capsule2D
    bool overlap(const core::vec::Vec2& point, const Capsule2D& cap);

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
