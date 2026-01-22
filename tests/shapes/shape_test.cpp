#include "shape_test.h"
#include <iostream>
#include <cassert>
#include <atlas/physics/shapes/shape.h>

using namespace atlas::physics::shape;

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
// Sphere-Box (AABB)
// -----------------------------
void testSphereBox() {
    Sphere s{{0, 0, 0}, 1.0f};
    Box b{{1.5f, 0, 0}, {0.5f, 0.5f, 0.5f}};
    Box b2{{3.0f, 0, 0}, {0.5f, 0.5f, 0.5f}};

    assert(overlap(s, b) && "Sphere-Box should overlap");
    assert(!overlap(s, b2) && "Sphere-Box should NOT overlap");
}

// -----------------------------
// Box-Box (AABB)
// -----------------------------
void testBoxBox() {
    Box b1{{0, 0, 0}, {1, 1, 1}};
    Box b2{{1.5f, 0, 0}, {1, 1, 1}};
    Box b3{{3.0f, 0, 0}, {1, 1, 1}};

    assert(overlap(b1, b2) && "Box-Box should overlap");
    assert(!overlap(b1, b3) && "Box-Box should NOT overlap");
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
// Capsule-Box
// -----------------------------
void testCapsuleBox() {
    Capsule cap{{-1, 0, 0}, {1, 0, 0}, 0.5f};
    Box b1{{0, 0, 0}, {0.5f, 0.5f, 0.5f}};
    Box b2{{0, 2, 0}, {0.5f, 0.5f, 0.5f}};

    assert(overlap(cap, b1) && "Capsule-Box should overlap");
    assert(!overlap(cap, b2) && "Capsule-Box should NOT overlap");
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
void testOBB() {
    // Basic OBB-AABB overlap test using axes-aligned OBB
    OBB obb1{{0, 0, 0}, {1, 1, 1}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Box b{{1.5f, 0, 0}, {0.5f, 0.5f, 0.5f}};
    Box b2{{3.0f, 0, 0}, {0.5f, 0.5f, 0.5f}};

    // Using AABB approximation for now
    assert(overlap(b, obb1) && "Box-OBB should overlap");
    assert(!overlap(b2, obb1) && "Box-OBB should NOT overlap");
}

int main() {
    testSphereSphere();
    testSphereBox();
    testBoxBox();
    testCapsuleSphere();
    testCapsuleBox();
    testCapsuleCapsule();
    testOBB();

    std::cout << "[shape] All tests passed.\n";
    return 0;
}
