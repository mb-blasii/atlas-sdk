#pragma once
#include <atlas/core/vectors/vec3.h>

namespace atlas::physics::shape {

#pragma region shapes

    struct Shape {
        void* ctx = nullptr; //Might be used for adding a shape context in the future
    };

    struct Sphere : Shape {
        core::vec::Vec3 center;
        float radius = 1.0f;

        Sphere() : Shape(nullptr) {}
        Sphere(const core::vec::Vec3& c, float r) : Shape(nullptr), center(c), radius(r) {}
    };

    struct Box : Shape {
        core::vec::Vec3 center;
        core::vec::Vec3 halfExtents;

        Box() : Shape(nullptr) {}
        Box(const core::vec::Vec3& c, const core::vec::Vec3& he) : Shape(nullptr), center(c), halfExtents(he) {}
    };

    struct OBB : Shape {
        core::vec::Vec3 center;
        core::vec::Vec3 halfExtents;
        core::vec::Vec3 axes[3]; // Local orientation axes (normalized)

        OBB() : Shape(nullptr) {}
        OBB(const core::vec::Vec3& c, const core::vec::Vec3& he, const core::vec::Vec3 a[3])
        : Shape(nullptr), center(c), halfExtents(he)
        {
            axes[0] = a[0];
            axes[1] = a[1];
            axes[2] = a[2];
        }
    };

    struct Capsule : Shape {
        core::vec::Vec3 a;
        core::vec::Vec3 b;
        float radius = 1.0f;

        Capsule() : Shape(nullptr) {}
        Capsule(const core::vec::Vec3& a, const core::vec::Vec3& b, float r) : Shape(nullptr), a(a), b(b), radius(r) {}
    };

#pragma endregion

#pragma region utility functions

    float distancePointSegment(const core::vec::Vec3& p, const core::vec::Vec3& a, const core::vec::Vec3& b);

    core::vec::Vec3 clampPointBox(const core::vec::Vec3& p, const Box& b);

    // OBB-SAT UTILITY
    bool overlapOnAxis(const OBB& a, const OBB& b, const core::vec::Vec3& axis);

#pragma endregion

#pragma region overlap

    //Point-Sphere
    bool overlap(const core::vec::Vec3& point, const Sphere& s);

    //Point-Box
    bool overlap(const core::vec::Vec3& point, const Box& b);

    //Point-OBB
    bool overlap(const core::vec::Vec3& point, const OBB& o);

    //Point-Capsule
    bool overlap(const core::vec::Vec3& point, const Capsule& c);

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

#pragma region compute AABB

    Box computeAABB(const Sphere& s, float scaleFactor = 1.0f);
    Box computeAABB(const Box& b, float scaleFactor = 1.0f);
    Box computeAABB(const OBB& o, float scaleFactor = 1.0f);
    Box computeAABB(const Capsule& c, float scaleFactor = 1.0f);

#pragma endregion

}
