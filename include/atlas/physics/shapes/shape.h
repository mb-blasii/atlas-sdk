#pragma once
#include <atlas/core/vectors/vec3.h>

namespace atlas::physics::shape {
    using namespace core::vec;

    struct Sphere {
        Vec3 center;
        float radius;
    };

    struct Box {
        Vec3 center;
        Vec3 halfExtents;
    };

    struct OBB {
        Vec3 center;
        Vec3 halfExtents;
        Vec3 axes[3]; // Local orientation axes (normalized)
    };

    struct Capsule {
        Vec3 a;
        Vec3 b;
        float radius;
    };

#pragma region utility functions

    float distancePointSegment(const Vec3& p, const Vec3& a, const Vec3& b);

    Vec3 clampPointBox(const Vec3& p, const Box& b);

    // OBB-SAT UTILITY
    bool overlapOnAxis(const OBB& a, const OBB& b, const Vec3& axis);

#pragma endregion

#pragma region overlap

    // Sphere-Sphere
    bool overlap(const Sphere& a, const Sphere& b);

    // Box-Box (AABB)
    bool overlap(const Box& a, const Box& b);

    // Capsule-Capsule
    bool overlap(const Capsule& a, const Capsule& b);

    // OBB-OBB using SAT
    bool overlap(const OBB& a, const OBB& b);

    // Sphere-Box (AABB)
    bool overlap(const Sphere& s, const Box& b);
    bool overlap(const Box& b, const Sphere& s);


    // Capsule-Sphere
    bool overlap(const Capsule& c, const Sphere& s);
    bool overlap(const Sphere& s, const Capsule& c);

    // Capsule-Box (AABB)
    bool overlap(const Capsule& c, const Box& b);
    bool overlap(const Box& b, const Capsule& c);

    // Sphere-OBB
    bool overlap(const Sphere& s, const OBB& o);
    bool overlap(const OBB& o, const Sphere& s);

    // Box(AABB)-OBB
    bool overlap(const Box& b, const OBB& o);
    bool overlap(const OBB& o, const Box& b);

    // Capsule-OBB
    bool overlap(const Capsule& c, const OBB& o);
    bool overlap(const OBB& o, const Capsule& c);

#pragma endregion

}
