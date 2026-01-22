#include <atlas/core/shapes/shape.h>

#include <cmath>
#include <algorithm>
#include <array>
#include <atlas/core/math/math.h>

namespace atlas::core::shape {
    using namespace vec;

#pragma region utility functions

    float distancePointSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
        Vec3 ab = b - a;
        float abLenSq = ab.lengthSq();
        if (abLenSq == 0.0f) return (p - a).length();

        float t = dot(p - a, ab) / abLenSq;
        t = std::clamp(t, 0.0f, 1.0f);
        Vec3 closest = a + ab * t;
        return (p - closest).length();
    }

    Vec3 clampPointBox(const Vec3& p, const Box& b) {
        Vec3 min = b.center - b.halfExtents;
        Vec3 max = b.center + b.halfExtents;
        return Vec3{
            std::clamp(p.x, min.x, max.x),
            std::clamp(p.y, min.y, max.y),
            std::clamp(p.z, min.z, max.z)
        };
    }

    // OBB-SAT UTILITY
    bool overlapOnAxis(const OBB& a, const OBB& b, const Vec3& axis) {
        if (math::isZero(axis.lengthSq())) return true; // skip near-zero axis

        // Project OBB a
        float aProj = 0.0f;
        for (int i = 0; i < 3; ++i) aProj += a.halfExtents[i] * std::abs(dot(a.axes[i], axis));

        // Project OBB b
        float bProj = 0.0f;
        for (int i = 0; i < 3; ++i) bProj += b.halfExtents[i] * std::abs(dot(b.axes[i], axis));

        float distanceCenters = std::abs(dot(b.center - a.center, axis));

        return distanceCenters <= aProj + bProj;
    }

#pragma endregion

#pragma region overlap

    // Sphere-Sphere
    bool overlap(const Sphere& a, const Sphere& b) {
        float r = a.radius + b.radius;
        return (a.center - b.center).lengthSq() <= r * r;
    }

    // Box-Box (AABB)
    bool overlap(const Box& a, const Box& b) {
        return std::abs(a.center.x - b.center.x) <= a.halfExtents.x + b.halfExtents.x &&
               std::abs(a.center.y - b.center.y) <= a.halfExtents.y + b.halfExtents.y &&
               std::abs(a.center.z - b.center.z) <= a.halfExtents.z + b.halfExtents.z;
    }

    // Capsule-Capsule
    bool overlap(const Capsule& a, const Capsule& b) {
        float d1 = distancePointSegment(a.a, b.a, b.b);
        float d2 = distancePointSegment(a.b, b.a, b.b);
        float d3 = distancePointSegment(b.a, a.a, a.b);
        float d4 = distancePointSegment(b.b, a.a, a.b);
        float minDist = std::min({d1, d2, d3, d4});
        return minDist <= a.radius + b.radius;
    }

    // OBB-OBB using SAT
    bool overlap(const OBB& a, const OBB& b) {
        std::array<Vec3, 15> axes;

        // 3 face normals from each OBB
        for (int i = 0; i < 3; ++i) axes[i] = a.axes[i];
        for (int i = 0; i < 3; ++i) axes[i + 3] = b.axes[i];

        // 9 cross products
        int idx = 6;
        for (auto axe : a.axes)
            for (auto j : b.axes)
                axes[idx++] = cross(axe, j);

        // Test all axes
        for (const Vec3& axis : axes) {
            if (!overlapOnAxis(a, b, axis))
                return false;
        }

        return true;
    }

    // Sphere-Box (AABB)
    bool overlap(const Sphere& s, const Box& b) {
        Vec3 closest = clampPointBox(s.center, b);
        return (closest - s.center).lengthSq() <= s.radius * s.radius;
    }
    bool overlap(const Box& b, const Sphere& s) { return overlap(s, b); }


    // Capsule-Sphere
    bool overlap(const Capsule& c, const Sphere& s) {
        return distancePointSegment(s.center, c.a, c.b) <= s.radius + c.radius;
    }
    bool overlap(const Sphere& s, const Capsule& c) { return overlap(c, s); }

    // Capsule-Box (AABB)
    bool overlap(const Capsule& c, const Box& b) {
        Vec3 closestPoint{};
        for (int i = 0; i < 3; ++i) {
            float minB = b.center[i] - b.halfExtents[i];
            float maxB = b.center[i] + b.halfExtents[i];
            float valA = std::clamp(c.a[i], minB, maxB);
            float valB = std::clamp(c.b[i], minB, maxB);
            closestPoint[i] = (valA + valB) * 0.5f;
        }
        return distancePointSegment(closestPoint, c.a, c.b) <= c.radius;
    }
    bool overlap(const Box& b, const Capsule& c) { return overlap(c, b); }

    // Sphere-OBB
    bool overlap(const Sphere& s, const OBB& o) {
        // Transform sphere center to OBB local space
        Vec3 local;
        Vec3 d = s.center - o.center;
        local.x = dot(d, o.axes[0]);
        local.y = dot(d, o.axes[1]);
        local.z = dot(d, o.axes[2]);

        // Clamp to OBB
        Vec3 closest{
            std::clamp(local.x, -o.halfExtents.x, o.halfExtents.x),
            std::clamp(local.y, -o.halfExtents.y, o.halfExtents.y),
            std::clamp(local.z, -o.halfExtents.z, o.halfExtents.z)
        };

        Vec3 diff = local - closest;
        return diff.lengthSq() <= s.radius * s.radius;
    }
    bool overlap(const OBB& o, const Sphere& s) { return overlap(s, o); }

    // Box(AABB)-OBB
    bool overlap(const Box& b, const OBB& o) {
        // Convert Box to OBB with axes = world axes
        OBB obbB;
        obbB.center = b.center;
        obbB.halfExtents = b.halfExtents;
        obbB.axes[0] = Vec3{1,0,0};
        obbB.axes[1] = Vec3{0,1,0};
        obbB.axes[2] = Vec3{0,0,1};
        return overlap(obbB, o);
    }
    bool overlap(const OBB& o, const Box& b) { return overlap(b, o); }

    // Capsule-OBB
    bool overlap(const Capsule& c, const OBB& o) {
        // Transform capsule endpoints into OBB local space
        auto toLocal = [&](const Vec3& p) -> Vec3 {
            Vec3 d = p - o.center;
            return Vec3{ dot(d, o.axes[0]), dot(d, o.axes[1]), dot(d, o.axes[2]) };
        };

        Vec3 localA = toLocal(c.a);
        Vec3 localB = toLocal(c.b);

        // Clamp each point to OBB bounds
        Vec3 clampedA{
            std::clamp(localA.x, -o.halfExtents.x, o.halfExtents.x),
            std::clamp(localA.y, -o.halfExtents.y, o.halfExtents.y),
            std::clamp(localA.z, -o.halfExtents.z, o.halfExtents.z)
        };
        Vec3 clampedB{
            std::clamp(localB.x, -o.halfExtents.x, o.halfExtents.x),
            std::clamp(localB.y, -o.halfExtents.y, o.halfExtents.y),
            std::clamp(localB.z, -o.halfExtents.z, o.halfExtents.z)
        };

        // If either endpoint is inside OBB, return true
        if ((clampedA - localA).lengthSq() <= c.radius * c.radius) return true;
        if ((clampedB - localB).lengthSq() <= c.radius * c.radius) return true;

        // Check distance from segment to OBB center
        Vec3 closestPoint{};
        for (int i = 0; i < 3; ++i) {
            float minO = -o.halfExtents[i];
            float maxO =  o.halfExtents[i];
            float valA = std::clamp(localA[i], minO, maxO);
            float valB = std::clamp(localB[i], minO, maxO);
            closestPoint[i] = (valA + valB) * 0.5f;
        }

        float dist = distancePointSegment(closestPoint, localA, localB);
        return dist <= c.radius;
    }
    bool overlap(const OBB& o, const Capsule& c) { return overlap(c, o); }

#pragma endregion

}
