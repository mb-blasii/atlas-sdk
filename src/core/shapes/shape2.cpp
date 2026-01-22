#include <atlas/core/shapes/shape2.h>

#include <cmath>
#include <algorithm>

namespace atlas::core::shape {
    using namespace vec;

#pragma region utility functions

    float distancePointSegmentSq(const Vec2 &p, const Vec2 &a, const Vec2 &b) {
        Vec2 ab = b - a;
        float abLenSq = ab.lengthSq();
        if (abLenSq == 0.0f) return (p - a).lengthSq();

        float t = dot(p - a, ab) / abLenSq;
        t = std::clamp(t, 0.0f, 1.0f);
        Vec2 closest = a + ab * t;
        return (p - closest).lengthSq();
    }

    Vec2 clampPointRect(const Vec2 &p, const Rect &r) {
        Vec2 min = r.center - r.halfExtents;
        Vec2 max = r.center + r.halfExtents;
        return {
            std::clamp(p.x, min.x, max.x),
            std::clamp(p.y, min.y, max.y)
        };
    }

#pragma endregion

#pragma region overlap

    // Rect-Rect
    bool overlap(const Rect &r1, const Rect &r2) {
        return std::abs(r1.center.x - r2.center.x) <= r1.halfExtents.x + r2.halfExtents.x &&
               std::abs(r1.center.y - r2.center.y) <= r1.halfExtents.y + r2.halfExtents.y;
    }

    // Circle-Circle
    bool overlap(const Circle &c1, const Circle &c2) {
        float r = c1.radius + c2.radius;
        return (c1.center - c2.center).lengthSq() <= r * r;
    }

    // Capsule-Capsule
    bool overlap(const Capsule2D &a, const Capsule2D &b) {
        float r = a.radius + b.radius;
        float d1 = distancePointSegmentSq(a.a, b.a, b.b);
        float d2 = distancePointSegmentSq(a.b, b.a, b.b);
        float d3 = distancePointSegmentSq(b.a, a.a, a.b);
        float d4 = distancePointSegmentSq(b.b, a.a, a.b);
        float minDistSq = std::min({d1, d2, d3, d4});
        return minDistSq <= r * r;
    }

    // Rect-Circle
    bool overlap(const Rect &r, const Circle &c) {
        Vec2 closest = clampPointRect(c.center, r);
        return (closest - c.center).lengthSq() <= c.radius * c.radius;
    }
    bool overlap(const Circle &c, const Rect &r) { return overlap(r, c); }

    // Capsule-Circle
    bool overlap(const Capsule2D &cap, const Circle &c) {
        float r = cap.radius + c.radius;
        return distancePointSegmentSq(c.center, cap.a, cap.b) <= r * r;
    }
    bool overlap(const Circle &c, const Capsule2D &cap) { return overlap(cap, c); }

    // Capsule-Rect
    bool overlap(const Capsule2D &cap, const Rect &r) {
        Vec2 ca = clampPointRect(cap.a, r);
        Vec2 cb = clampPointRect(cap.b, r);

        // If either endpoint is within radius of rect, overlap exists
        if ((ca - cap.a).lengthSq() <= cap.radius * cap.radius) return true;
        if ((cb - cap.b).lengthSq() <= cap.radius * cap.radius) return true;

        // Check capsule segment vs rect center
        float distSq = distancePointSegmentSq(r.center, cap.a, cap.b);
        return distSq <= cap.radius * cap.radius;
    }
    bool overlap(const Rect &r, const Capsule2D &cap) { return overlap(cap, r); }

#pragma endregion

}