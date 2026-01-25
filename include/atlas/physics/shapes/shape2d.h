#pragma once
#include <atlas/core/vectors/vec2.h>

namespace atlas::physics::shape {

    struct Rect {
        core::vec::Vec2 center;
        core::vec::Vec2 halfExtents;
    };

    struct Shape2D {
        void* ctx = nullptr; //context: might be used in future to bind a shape with an owning object
        virtual ~Shape2D() = default;

        virtual Rect computeRect(float scaleFactor = 1.0f) const = 0;
    };

    struct Circle : Shape2D {
        core::vec::Vec2 center;
        float radius = 1.0f;

        Circle() = default;
        Circle(const core::vec::Vec2& c, float r) : center(c), radius(r) {}

        Rect computeRect(float scaleFactor = 1.0f) const override;
    };

    struct Capsule2D : Shape2D {
        core::vec::Vec2 a;
        core::vec::Vec2 b;
        float radius = 1.0f;

        Capsule2D() = default;
        Capsule2D(const core::vec::Vec2& a, const core::vec::Vec2& b, float r) : a(a), b(b), radius(r) {}

        Rect computeRect(float scaleFactor = 1.0f) const override;
    };

    // OBB 2D
    struct Box2D : Shape2D {
        core::vec::Vec2 center;
        core::vec::Vec2 halfExtents;
        core::vec::Vec2 axes[2]; // normalized, orthogonal

        Rect computeRect(float scaleFactor = 1.0f) const override;
    };

#pragma region utility functions

    float distancePointSegmentSq(const core::vec::Vec2 &p, const core::vec::Vec2 &a, const core::vec::Vec2 &b);
    float distanceSegmentRectSq(const core::vec::Vec2& a, const core::vec::Vec2& b, const Rect& r);

    core::vec::Vec2 clampPointRect(const core::vec::Vec2 &p, const Rect &r);

#pragma endregion

#pragma region overlap

    // Point-Circle
    bool overlap(const core::vec::Vec2& point, const Circle& c);

    // Point-Rect
    bool overlap(const core::vec::Vec2& point, const Rect& r);

    // Point-Capsule2D
    bool overlap(const core::vec::Vec2& point, const Capsule2D& cap);

    // Point-Box2D
    bool overlap(const core::vec::Vec2& point, const Box2D& b);

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

    //Box2D-Rects
    bool overlap(const Box2D& b, const Rect& r);
    bool overlap(const Rect& r, const Box2D& b);

    // Box2D-Box2D
    bool overlap(const Box2D& a, const Box2D& b);

    // Box2D-Circle
    bool overlap(const Box2D& b, const Circle& c);
    bool overlap(const Circle& c, const Box2D& b);

    // Box2D-Capsule
    bool overlap(const Box2D& b, const Capsule2D& cap);
    bool overlap(const Capsule2D& cap, const Box2D& b);

#pragma endregion

}
