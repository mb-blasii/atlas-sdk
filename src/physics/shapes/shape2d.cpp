#include <atlas/physics/shapes/shape2d.h>

#include <cmath>
#include <algorithm>

namespace atlas::physics::shape {
    using namespace core::vec;

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

    float distanceSegmentRectSq(const Vec2& a, const Vec2& b, const Rect& r)
    {
        Vec2 d = b - a;

        float tMin = 0.0f;
        float tMax = 1.0f;

        Vec2 min = r.center - r.halfExtents;
        Vec2 max = r.center + r.halfExtents;

        // Clipping Liang–Barsky
        for (int i = 0; i < 2; ++i)
        {
            float p = (i == 0) ? d.x : d.y;
            float q0 = ((i == 0) ? a.x : a.y) - ((i == 0) ? min.x : min.y);
            float q1 = ((i == 0) ? max.x : max.y) - ((i == 0) ? a.x : a.y);

            if (fabs(p) < 1e-6f)
            {
                if (q0 < 0.0f || q1 < 0.0f)
                    goto outside;
            }
            else
            {
                float t0 = q0 / p;
                float t1 = q1 / p;
                if (t0 > t1) std::swap(t0, t1);

                tMin = std::max(tMin, t0);
                tMax = std::min(tMax, t1);

                if (tMin > tMax)
                    goto outside;
            }
        }

        // Segment-Rect intersection
        return 0.0f;

        outside:
            // closest point to Rect
            float t = std::clamp(tMin, 0.0f, 1.0f);
        Vec2 p = a + d * t;

        Vec2 cp = clampPointRect(p, r);
        return (p - cp).lengthSq();
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

#pragma region compute Rect

    Rect Circle::computeRect(float scaleFactor) const {
        Vec2 he(radius, radius);
        he *= scaleFactor;
        return Rect{center, he};
    }

    Rect Capsule2D::computeRect(float scaleFactor) const {
        Vec2 minP{
            std::min(a.x, b.x),
            std::min(a.y, b.y)
        };

        Vec2 maxP{
            std::max(a.x, b.x),
            std::max(a.y, b.y)
        };

        Vec2 r(radius, radius);
        minP -= r;
        maxP += r;

        Vec2 center = (minP + maxP) * 0.5f;
        Vec2 halfExtents = (maxP - minP) * 0.5f;
        halfExtents *= scaleFactor;

        return Rect{center, halfExtents};
    }

    Rect Box2D::computeRect(float scaleFactor) const {
        Vec2 he;
        he.x = std::abs(axes[0].x) * halfExtents.x +
               std::abs(axes[1].x) * halfExtents.y;
        he.y = std::abs(axes[0].y) * halfExtents.x +
               std::abs(axes[1].y) * halfExtents.y;

        he *= scaleFactor;
        return Rect{center, he};
    }

#pragma endregion

#pragma region overlap

    // Point-Circle
    bool overlap(const Vec2& point, const Circle& c) {
        Vec2 diff = point - c.center;
        return diff.lengthSq() <= c.radius * c.radius;
    }

    // Point-Rect
    bool overlap(const Vec2& point, const Rect& r) {
        Vec2 minR = r.center - r.halfExtents;
        Vec2 maxR = r.center + r.halfExtents;
        return point.x >= minR.x && point.x <= maxR.x &&
               point.y >= minR.y && point.y <= maxR.y;
    }

    // Point-Capsule2D
    bool overlap(const Vec2& point, const Capsule2D& cap) {
        float distSq = distancePointSegmentSq(point, cap.a, cap.b);
        return distSq <= cap.radius * cap.radius;
    }

    // Point-Box2D
    bool overlap(const Vec2& point, const Box2D& b) {
        Vec2 local = point - b.center;
        for (int i = 0; i < 2; ++i) {
            float dist = dot(local, b.axes[i]);
            if (dist < -b.halfExtents[i] || dist > b.halfExtents[i])
                return false;
        }
        return true;
    }

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

        if ((ca - cap.a).lengthSq() <= cap.radius * cap.radius) return true;
        if ((cb - cap.b).lengthSq() <= cap.radius * cap.radius) return true;

        float distSq = distancePointSegmentSq(r.center, cap.a, cap.b);
        return distSq <= cap.radius * cap.radius;
    }
    bool overlap(const Rect &r, const Capsule2D &cap) { return overlap(cap, r); }

    // Box2D-Rect
    bool overlap(const Box2D& b, const Rect& r)
    {
        // Trasformazione in spazio OBB
        Vec2 d = r.center - b.center;

        Vec2 local;
        local.x = dot(d, b.axes[0]);
        local.y = dot(d, b.axes[1]);

        Vec2 ext;
        ext.x =
            r.halfExtents.x * fabs(dot(b.axes[0], {1,0})) +
            r.halfExtents.y * fabs(dot(b.axes[0], {0,1}));

        ext.y =
            r.halfExtents.x * fabs(dot(b.axes[1], {1,0})) +
            r.halfExtents.y * fabs(dot(b.axes[1], {0,1}));

        if (fabs(local.x) > b.halfExtents.x + ext.x) return false;
        if (fabs(local.y) > b.halfExtents.y + ext.y) return false;

        return true;
    }

    bool overlap(const Rect& r, const Box2D& b) { return overlap(b, r); }

    //Box-Box
    bool overlap(const Box2D& a, const Box2D& b)
    {
        Vec2 axes[4] = {
            a.axes[0],
            a.axes[1],
            b.axes[0],
            b.axes[1]
        };

        Vec2 d = b.center - a.center;

        for (Vec2 axis : axes)
        {
            axis.normalize();

            float aProj =
                fabs(dot(axis, a.axes[0])) * a.halfExtents.x +
                fabs(dot(axis, a.axes[1])) * a.halfExtents.y;

            float bProj =
                fabs(dot(axis, b.axes[0])) * b.halfExtents.x +
                fabs(dot(axis, b.axes[1])) * b.halfExtents.y;

            float dist = fabs(dot(d, axis));

            if (dist > aProj + bProj)
                return false;
        }

        return true;
    }

    // Box2D-Circle
    bool overlap(const Box2D& b, const Circle& c) {
        Vec2 d = c.center - b.center;
        Vec2 local{
            dot(d, b.axes[0]),
            dot(d, b.axes[1])
        };

        Vec2 closest{
            std::clamp(local.x, -b.halfExtents.x, b.halfExtents.x),
            std::clamp(local.y, -b.halfExtents.y, b.halfExtents.y)
        };

        Vec2 diff = local - closest;
        return diff.lengthSq() <= c.radius * c.radius;
    }
    bool overlap(const Circle& c, const Box2D& b) { return overlap(b, c); }

    // Box2D-Capsule
    bool overlap(const Box2D& b, const Capsule2D& cap)
    {
        // Trasforma capsula in spazio locale OBB
        auto toLocal = [&](const Vec2& p) {
            Vec2 d = p - b.center;
            return Vec2{
                dot(d, b.axes[0]),
                dot(d, b.axes[1])
            };
        };

        Vec2 a = toLocal(cap.a);
        Vec2 c = toLocal(cap.b);

        Rect localRect;
        localRect.center = Vec2{ 0.0f, 0.0f };
        localRect.halfExtents = b.halfExtents;

        float distSq = distanceSegmentRectSq(a, c, localRect);
        return distSq <= cap.radius * cap.radius;
    }
    bool overlap(const Capsule2D& cap, const Box2D& b) { return overlap(b, cap); }

#pragma endregion

}
