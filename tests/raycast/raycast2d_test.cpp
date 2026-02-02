#include "raycast2d_test.h"
#include <atlas/physics/raycast/raycast2d.h>
#include <atlas/core/math/math.h>
#include <cassert>
#include <iostream>

using namespace atlas::physics::ray;
using namespace atlas::physics::shape;
using namespace atlas::core::vec;
using namespace atlas::core::math;

//Helper
void assertVec2NearlyEqual(const Vec2& a, const Vec2& b) {
    assert(nearlyEqual(a.x, b.x) && "Point X should be equal");
    assert(nearlyEqual(a.y, b.y) && "Point Y should be equal");
}

//Circle
void testRaycast2DCircle() {
    Circle c{ Vec2{0,0}, 1.0f };
    RayResult2D r;

    // 1. No collision
    {
        Ray2D ray{ Vec2{-5, 0}, Vec2{0, 1} };
        assert(!raycast(ray, c, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray2D ray{ Vec2{0,0}, Vec2{1,0} };
        assert(raycast(ray, c, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
        assertVec2NearlyEqual(r.point, Vec2{0,0});
    }

    // 3. Correct collision
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        assert(raycast(ray, c, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        raycast(ray, c, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.0f) && "Distance should be equal");
        assertVec2NearlyEqual(r.point, Vec2{-1,0});
        assertVec2NearlyEqual(r.normal, Vec2{-1,0});
    }
}

//Rect
void testRaycast2DRect() {
    Rect rct{ Vec2{0,0}, Vec2{1,1} };
    RayResult2D r;

    // 1. No collision
    {
        Ray2D ray{ Vec2{5,0}, Vec2{1,0} };
        assert(!raycast(ray, rct, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray2D ray{ Vec2{0,0}, Vec2{1,0} };
        assert(raycast(ray, rct, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
        assertVec2NearlyEqual(r.point, Vec2{0,0});
    }

    // 3. Correct collision
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        assert(raycast(ray, rct, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        raycast(ray, rct, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.0f) && "Distance should be equal");
        assertVec2NearlyEqual(r.point, Vec2{-1,0});
        assertVec2NearlyEqual(r.normal, Vec2{-1,0});
    }
}

//Capsule2D
void testRaycast2DCapsule() {
    Capsule2D cap{ Vec2{0,-1}, Vec2{0,1}, 0.5f };
    RayResult2D r;

    // 1. No collision
    {
        Ray2D ray{ Vec2{5,0}, Vec2{1,0} };
        assert(!raycast(ray, cap, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray2D ray{ Vec2{0,0}, Vec2{1,1} };
        assert(raycast(ray, cap, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
        assertVec2NearlyEqual(r.point, Vec2{0,0});
    }

    // 3. Correct collision
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        assert(raycast(ray, cap, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray2D ray{ Vec2{-5,0}, Vec2{1,0} };
        raycast(ray, cap, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.5f) && "Distance should be equal");
        assertVec2NearlyEqual(r.point, Vec2{-0.5f,0});
        assertVec2NearlyEqual(r.normal, Vec2{-1,0});
    }
}

int main() {
    testRaycast2DCircle();
    testRaycast2DRect();
    testRaycast2DCapsule();

    std::cout << "[raycast2d] All tests passed successfully.\n";
    return 0;
}
