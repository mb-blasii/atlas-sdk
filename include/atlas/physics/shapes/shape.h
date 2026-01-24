#pragma once
#include <atlas/core/vectors/vec3.h>

namespace atlas::physics::shape {

#pragma region shapes

    struct AABB {
        core::vec::Vec3 center;
        core::vec::Vec3 halfExtents;
    };

    struct Shape {
        void* ctx = nullptr; //Might be used for adding a shape context in the future

        Shape() = default;
        explicit Shape(void* ctx) : ctx(ctx) {}
        virtual ~Shape() = default;

        virtual AABB computeAABB(float scaleFactor = 1.0f) const;
    };

    struct Sphere : Shape {
        core::vec::Vec3 center;
        float radius = 1.0f;

        Sphere() : Shape(nullptr) {}
        Sphere(const core::vec::Vec3& c, float r) : Shape(nullptr), center(c), radius(r) {}

        AABB computeAABB(float scaleFactor = 1.0f) const override;
    };


    struct Box : Shape {
        core::vec::Vec3 center;
        core::vec::Vec3 halfExtents;
        core::vec::Vec3 axes[3]; // Local orientation axes (normalized)

        Box() : Shape(nullptr) {}
        Box(const core::vec::Vec3& c, const core::vec::Vec3& he, const core::vec::Vec3 a[3])
        : Shape(nullptr), center(c), halfExtents(he)
        {
            axes[0] = a[0];
            axes[1] = a[1];
            axes[2] = a[2];
        }

        AABB computeAABB(float scaleFactor = 1.0f) const override;
    };

    struct Capsule : Shape {
        core::vec::Vec3 a;
        core::vec::Vec3 b;
        float radius = 1.0f;

        Capsule() : Shape(nullptr) {}
        Capsule(const core::vec::Vec3& a, const core::vec::Vec3& b, float r) : Shape(nullptr), a(a), b(b), radius(r) {}

        AABB computeAABB(float scaleFactor = 1.0f) const override;
    };

#pragma endregion

#pragma region utility functions

    float distancePointSegment(const core::vec::Vec3& p, const core::vec::Vec3& a, const core::vec::Vec3& b);

    core::vec::Vec3 clampPointAABB(const core::vec::Vec3& p, const AABB& b);

    // OBB-SAT UTILITY
    bool overlapOnAxis(const Box& a, const Box& b, const core::vec::Vec3& axis);

#pragma endregion

#pragma region overlap

    //Point-Sphere
    bool overlap(const core::vec::Vec3& point, const Sphere& s);

    //Point-AABB
    bool overlap(const core::vec::Vec3& point, const AABB& b);

    //Point-Box
    bool overlap(const core::vec::Vec3& point, const Box& o);

    //Point-Capsule
    bool overlap(const core::vec::Vec3& point, const Capsule& c);

    // Sphere-Sphere
    bool overlap(const Sphere& a, const Sphere& b);

    // AABB
    bool overlap(const AABB& a, const AABB& b);

    // Capsule-Capsule
    bool overlap(const Capsule& a, const Capsule& b);

    // Box-Box using SAT
    bool overlap(const Box& a, const Box& b);

    // Sphere-AABB
    bool overlap(const Sphere& s, const AABB& b);
    bool overlap(const AABB& b, const Sphere& s);

    // Capsule-Sphere
    bool overlap(const Capsule& c, const Sphere& s);
    bool overlap(const Sphere& s, const Capsule& c);

    // Capsule-AABB
    bool overlap(const Capsule& c, const AABB& b);
    bool overlap(const AABB& b, const Capsule& c);

    // Sphere-Box
    bool overlap(const Sphere& s, const Box& o);
    bool overlap(const Box& o, const Sphere& s);

    // AABB-Box
    bool overlap(const AABB& b, const Box& o);
    bool overlap(const Box& o, const AABB& b);

    // Capsule-Box
    bool overlap(const Capsule& c, const Box& o);
    bool overlap(const Box& o, const Capsule& c);

#pragma endregion

#pragma region compute AABB

    AABB sphereAABB(const Sphere& s, float scaleFactor = 1.0f);
    AABB boxAABB(const Box& o, float scaleFactor = 1.0f);
    AABB capsuleAABB(const Capsule& c, float scaleFactor = 1.0f);

#pragma endregion

}
