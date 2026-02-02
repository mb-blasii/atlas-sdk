#pragma once
#include <atlas/core/vectors/vec2.h>
#include <atlas/physics/shapes/shape2d.h>

namespace atlas::physics::ray {

    struct Ray2D {
        core::vec::Vec2 origin;
        core::vec::Vec2 direction; //Normalized in raycast functions
    };

    struct RayResult2D {
        bool hit = false;
        core::vec::Vec2 point{0,0};
        core::vec::Vec2 normal{0,0};
        float distance = 0.0f;
    };

    bool raycast(const Ray2D& ray, const shape::Circle& c, RayResult2D& out);
    bool raycast(const Ray2D& ray, const shape::Rect& r, RayResult2D& out);
    bool raycast(const Ray2D& ray, const shape::Capsule2D& cap, RayResult2D& out);

}
