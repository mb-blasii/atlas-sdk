#include "shape_test.h"
#include <iostream>
#include <cassert>
#include <atlas/physics/shapes/shape.h>

#include <atlas/core/math/math.h>

using namespace atlas::physics::shape;
using namespace atlas::core::vec;
using namespace atlas::core::math;

// -----------------------------
// Point-Sphere
// -----------------------------
void testPointSphere() {
    Sphere s{Vec3{0, 0, 0}, 1.0f};

    // 1. Outside point
    assert(!overlap(Vec3{2,0,0}, s) && "Point should not be inside sphere");

    // 2. Center point (inside)
    assert(overlap(Vec3{0,0,0}, s) && "Point at center should be inside");

    // 3. Surface point (inside)
    assert(overlap(Vec3{1,0,0}, s) && "Point on surface should be inside");

    // 4. Slightly outside
    assert(!overlap(Vec3{1.01f,0,0}, s) && "Point slightly outside should not be inside");
}

// -----------------------------
// Point-AABB
// -----------------------------
void testPointAABB() {
    AABB b{Vec3{0, 0, 0}, Vec3{1, 1, 1}};

    // 1. Outside
    assert(!overlap(Vec3{2,0,0}, b) && "Point should not be inside box");

    // 2. Inside
    assert(overlap(Vec3{0,0,0}, b) && "Point at center should be inside");

    // 3. On surface
    assert(overlap(Vec3{1,0,0}, b) && "Point on surface should be inside");

    // 4. Slightly outside
    assert(!overlap(Vec3{1.01f,0,0}, b) && "Point slightly outside should not be inside");
}

// -----------------------------
// Point-OBB
// -----------------------------
void testPointBox() {
    Box o;
    o.center = Vec3{0, 0, 0};
    o.halfExtents = Vec3{1, 1, 1};
    o.axes[0] = Vec3{1, 0, 0};
    o.axes[1] = Vec3{0, 1, 0};
    o.axes[2] = Vec3{0, 0, 1};

    // 1. Outside
    assert(!overlap(Vec3{2,0,0}, o) && "Point should not be inside OBB");

    // 2. Center
    assert(overlap(Vec3{0,0,0}, o) && "Point at center should be inside OBB");

    // 3. Surface
    assert(overlap(Vec3{1,0,0}, o) && "Point on surface should be inside OBB");

    // 4. Slightly outside
    assert(!overlap(Vec3{1.01f,0,0}, o) && "Point slightly outside should not be inside OBB");
}

// -----------------------------
// Point-Capsule
// -----------------------------
void testPointCapsule() {
    Capsule c;
    c.a = Vec3{0, -1, 0};
    c.b = Vec3{0, 1, 0};
    c.radius = 0.5f;

    // 1. Outside
    assert(!overlap(Vec3{1,0,0}, c) && "Point should not be inside capsule");

    // 2. Center
    assert(overlap(Vec3{0,0,0}, c) && "Point at center should be inside capsule");

    // 3. On surface (radius boundary)
    assert(overlap(Vec3{0.5f,0,0}, c) && "Point on surface should be inside capsule");

    // 4. Slightly outside
    assert(!overlap(Vec3{0.51f,0,0}, c) && "Point slightly outside should not be inside capsule");
}

// -----------------------------
// Sphere-Sphere
// -----------------------------
void testSphereSphere() {
    Sphere s1{{0, 0, 0}, 1.0f};
    Sphere s2{{1.5f, 0, 0}, 1.0f};
    Sphere s3{{3.0f, 0, 0}, 1.0f};

    assert(overlap(s1, s2) && "Sphere-Sphere should overlap");
    assert(!overlap(s1, s3) && "Sphere-Sphere should NOT overlap");
}

// -----------------------------
// Sphere-AABB
// -----------------------------
void testSphereAABB() {
    Sphere s{{0, 0, 0}, 1.0f};
    AABB b{{1.5f, 0, 0}, {0.5f, 0.5f, 0.5f}};
    AABB b2{{3.0f, 0, 0}, {0.5f, 0.5f, 0.5f}};

    assert(overlap(s, b) && "Sphere-AABB should overlap");
    assert(!overlap(s, b2) && "Sphere-AABB should NOT overlap");
}

// -----------------------------
// AABB
// -----------------------------
void testAABB() {
    AABB b1{{0, 0, 0}, {1, 1, 1}};
    AABB b2{{1.5f, 0, 0}, {1, 1, 1}};
    AABB b3{{3.0f, 0, 0}, {1, 1, 1}};

    assert(overlap(b1, b2) && "AABB-AABB should overlap");
    assert(!overlap(b1, b3) && "AABB-AABB should NOT overlap");
}

// -----------------------------
// Capsule-Sphere
// -----------------------------
void testCapsuleSphere() {
    Capsule cap{{-1, 0, 0}, {1, 0, 0}, 0.5f};
    Sphere s1{{0, 0.2f, 0}, 0.3f};
    Sphere s2{{0, 2.0f, 0}, 0.3f};

    assert(overlap(cap, s1) && "Capsule-Sphere should overlap");
    assert(!overlap(cap, s2) && "Capsule-Sphere should NOT overlap");
}

// -----------------------------
// Capsule-AABB
// -----------------------------
void testCapsuleAABB() {
    Capsule cap{{-1, 0, 0}, {1, 0, 0}, 0.5f};
    AABB b1{{0, 0, 0}, {0.5f, 0.5f, 0.5f}};
    AABB b2{{0, 2, 0}, {0.5f, 0.5f, 0.5f}};

    assert(overlap(cap, b1) && "Capsule-AABB should overlap");
    assert(!overlap(cap, b2) && "Capsule-AABB should NOT overlap");
}

// -----------------------------
// Capsule-Capsule
// -----------------------------
void testCapsuleCapsule() {
    Capsule c1{{-1, 0, 0}, {1, 0, 0}, 0.5f};
    Capsule c2{{-1, 0.4f, 0}, {1, 0.4f, 0}, 0.5f};
    Capsule c3{{-1, 3.0f, 0}, {1, 3.0f, 0}, 0.5f};

    assert(overlap(c1, c2) && "Capsule-Capsule should overlap");
    assert(!overlap(c1, c3) && "Capsule-Capsule should NOT overlap");
}

// -----------------------------
// OBB tests (minimal)
// -----------------------------
void testBox() {
    // Basic OBB-AABB overlap test using axes-aligned OBB
    Vec3 axes[3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    Box obb1{{0, 0, 0}, {1, 1, 1}, axes};
    AABB b{{1.5f, 0, 0}, {0.5f, 0.5f, 0.5f}};
    AABB b2{{3.0f, 0, 0}, {0.5f, 0.5f, 0.5f}};

    // Using AABB approximation for now
    assert(overlap(b, obb1) && "Box-OBB should overlap");
    assert(!overlap(b2, obb1) && "Box-OBB should NOT overlap");
}

//Test compute AABBs

void assertVec3Equal(const Vec3& a, const Vec3& b, const char* msg = "") {
    assert(nearlyEqual(a.x, b.x) && msg);
    assert(nearlyEqual(a.y, b.y) && msg);
    assert(nearlyEqual(a.z, b.z) && msg);
}

void assertAABBEqual(const AABB& a, const AABB& b, const char* msg = "") {
    assertVec3Equal(a.center, b.center, msg);
    assertVec3Equal(a.halfExtents, b.halfExtents, msg);
}

void testComputeAABB_sphere() {
    Sphere s({0.f, 0.f, 0.f}, 1.f);

    AABB aabb = s.computeAABB();

    AABB expected(
        {0.f, 0.f, 0.f},
        {1.f, 1.f, 1.f}
    );

    assertAABBEqual(aabb, expected);
}

void testComputeAABB_box() {
    Vec3 axes[3] = {
        {1.f, 0.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f}
    };

    Box obb(
        {0.f, 0.f, 0.f},
        {1.f, 2.f, 3.f},
        axes
    );

    AABB aabb = obb.computeAABB();

    AABB expected(
        {0.f, 0.f, 0.f},
        {1.f, 2.f, 3.f}
    );

    assertAABBEqual(aabb, expected);
}

void testComputeAABB_capsule() {
    Capsule c(
        {0.f, 0.f, 0.f},
        {0.f, 2.f, 0.f},
        0.5f
    );

    AABB aabb = c.computeAABB();

    AABB expected(
        {0.f, 1.f, 0.f},
        {0.5f, 1.5f, 0.5f}
    );

    assertAABBEqual(aabb, expected);
}

int main() {

    testPointSphere();
    testPointAABB();
    testPointBox();
    testPointCapsule();
    testSphereSphere();
    testSphereAABB();
    testAABB();
    testCapsuleSphere();
    testCapsuleAABB();
    testCapsuleCapsule();
    testBox();

    //Test AABBs
    testComputeAABB_sphere();
    testComputeAABB_box();
    testComputeAABB_capsule();

    std::cout << "[shapes] All tests passed.\n";
    return 0;
}
