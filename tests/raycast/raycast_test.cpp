#include "raycast_test.h"

#include <atlas/physics/raycast/raycast.h>
#include <atlas/core/math/math.h>

#include <cassert>
#include <iostream>

using namespace atlas::physics::ray;
using namespace atlas::physics::shape;
using namespace atlas::core::vec;
using namespace atlas::core::math;

// Helper
static void assertVec3NearlyEqual(const Vec3& a, const Vec3& b) {
    assert(nearlyEqual(a.x, b.x) && "Point X should be equal");
    assert(nearlyEqual(a.y, b.y) && "Point Y should be equal");
    assert(nearlyEqual(a.z, b.z) && "Point Z should be equal");
}

// Sphere
void testRaycastSphere() {
    Sphere s{ Vec3{0, 0, 0}, 1.0f };
    RayResult r;

    // 1. No collision
    {
        Ray ray{ Vec3{0, 0, -5}, Vec3{0, 1, 0} };
        assert(!raycast(ray, s, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray ray{ Vec3{0, 0, 0}, Vec3{1, 0, 0} };
        assert(raycast(ray, s, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
    }

    // 3. Correct collision
    {
        Ray ray{ Vec3{0, 0, -5}, Vec3{0, 0, 1} };
        assert(raycast(ray, s, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray ray{ Vec3{0, 0, -5}, Vec3{0, 0, 1} };
        raycast(ray, s, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.0f) && "Distance should be equal");
        assertVec3NearlyEqual(r.point, Vec3{0, 0, -1});
        assertVec3NearlyEqual(r.normal, Vec3{0, 0, -1});
    }
}

// Box (AABB)
void testRaycastBox() {
    Box b{ Vec3{0, 0, 0}, Vec3{1, 1, 1} };
    RayResult r;

    // 1. No collision
    {
        Ray ray{ Vec3{5, 0, 0}, Vec3{1, 0, 0} };
        assert(!raycast(ray, b, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray ray{ Vec3{0, 0, 0}, Vec3{1, 0, 0} };
        assert(raycast(ray, b, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
    }

    // 3. Correct collision
    {
        Ray ray{ Vec3{-5, 0, 0}, Vec3{1, 0, 0} };
        assert(raycast(ray, b, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray ray{ Vec3{-5, 0, 0}, Vec3{1, 0, 0} };
        raycast(ray, b, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.0f) && "Distance should be equal");
        assertVec3NearlyEqual(r.point, Vec3{-1, 0, 0});
        assertVec3NearlyEqual(r.normal, Vec3{-1, 0, 0});
    }
}

// OBB
void testRaycastOBB() {
    OBB o;
    o.center = Vec3{0, 0, 0};
    o.halfExtents = Vec3{1, 1, 1};
    o.axes[0] = Vec3{1, 0, 0};
    o.axes[1] = Vec3{0, 1, 0};
    o.axes[2] = Vec3{0, 0, 1};

    RayResult r;

    // 1. No collision
    {
        Ray ray{ Vec3{0, 5, 0}, Vec3{0, 1, 0} };
        assert(!raycast(ray, o, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray ray{ Vec3{0, 0, 0}, Vec3{0, 1, 0} };
        assert(raycast(ray, o, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
    }

    // 3. Correct collision
    {
        Ray ray{ Vec3{0, -5, 0}, Vec3{0, 1, 0} };
        assert(raycast(ray, o, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray ray{ Vec3{0, -5, 0}, Vec3{0, 1, 0} };
        raycast(ray, o, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.0f) && "Distance should be equal");
        assertVec3NearlyEqual(r.point, Vec3{0, -1, 0});
        assertVec3NearlyEqual(r.normal, Vec3{0, -1, 0});
    }
}

// Capsule
void testRaycastCapsule() {
    Capsule c;
    c.a = Vec3{0, -1, 0};
    c.b = Vec3{0,  1, 0};
    c.radius = 0.5f;

    RayResult r;

    // 1. No collision
    {
        Ray ray{ Vec3{5, 0, 0}, Vec3{1, 0, 0} };
        assert(!raycast(ray, c, r) && "Ray should not collide");
    }

    // 2. Internal collision
    {
        Ray ray{ Vec3{0, 0, 0}, Vec3{1, 0, 0} };
        assert(raycast(ray, c, r) && "Internal ray should collide");
        assert(r.hit && "Ray should have a hit");
    }

    // 3. Correct collision
    {
        Ray ray{ Vec3{-5, 0, 0}, Vec3{1, 0, 0} };
        assert(raycast(ray, c, r) && "Ray should collide");
    }

    // 4. Result test
    {
        Ray ray{ Vec3{-5, 0, 0}, Vec3{1, 0, 0} };
        raycast(ray, c, r);

        assert(r.hit && "Ray should have hit");
        assert(nearlyEqual(r.distance, 4.5f) && "Distance should be equal");
        assertVec3NearlyEqual(r.point, Vec3{-0.5f, 0, 0});
        assertVec3NearlyEqual(r.normal, Vec3{-1, 0, 0});
    }
}

int main() {
    testRaycastSphere();
    testRaycastBox();
    testRaycastOBB();
    testRaycastCapsule();

    std::cout << "[raycast] All tests passed successfully.\n";
    return 0;
}
