#pragma once

#include <atlas/core/vectors/vec3.h>
#include <atlas/physics/shapes/shape.h>

namespace atlas::physics::ray {

    struct Ray {
        core::vec::Vec3 origin;
        core::vec::Vec3 direction; //normalized in functions
    };

    struct RayResult {
        bool hit = false;
        float distance = 0.0f;
        core::vec::Vec3 point{};
        core::vec::Vec3 normal{};
    };

    // Raycast (single hit, closest)
    bool raycast(const Ray& ray, const shape::Sphere& s, RayResult& out);
    bool raycast(const Ray& ray, const shape::Box& b, RayResult& out);
    bool raycast(const Ray& ray, const shape::OBB& o, RayResult& out);
    bool raycast(const Ray& ray, const shape::Capsule& c, RayResult& out);

}
