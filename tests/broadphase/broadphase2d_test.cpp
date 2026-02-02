#include "broadphase2d_test.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

#include <atlas/physics/broadphase/broadphase2d.h>
#include <atlas/physics/shapes/shape2d.h>
#include <atlas/physics/raycast/raycast2d.h>
#include <atlas/core/vectors/vec2.h>

using namespace atlas;
using namespace atlas::physics;
using namespace atlas::physics::shape;
using namespace atlas::physics::bp;

bool containsCtx(const std::vector<Shape2D*>& shapes, const char* name) {
    for (auto* s : shapes) {
        if (std::strcmp(static_cast<const char*>(s->ctx), name) == 0)
            return true;
    }
    return false;
}

void testShapeCandidatesBasic2D() {
    Broadphase2D bp(1.0f);

    Circle a({0, 0}, 1.0f);
    Circle b({1.5f, 0}, 1.0f);
    Circle c({5, 0}, 1.0f);

    a.ctx = (void*)"Shape A";
    b.ctx = (void*)"Shape B";
    c.ctx = (void*)"Shape C";

    bp.update(&a);
    bp.update(&b);
    bp.update(&c);

    auto candidates = bp.getCandidates(&a);

    assert(candidates.size() == 1 &&
        "Expected exactly one candidate for Shape A");

    assert(containsCtx(candidates, "Shape B") &&
        "Expected Shape B as candidate");

    assert(!containsCtx(candidates, "Shape A") &&
        "Query shape must not return itself");

    assert(!containsCtx(candidates, "Shape C") &&
        "Far shape must not be returned");
}

void testTouchingShapes2D() {
    Broadphase2D bp(1.0f);

    Circle a({0, 0}, 1.0f);
    Circle b({2.0f, 0}, 1.0f); // exactly touching

    a.ctx = (void*)"A";
    b.ctx = (void*)"B";

    bp.update(&a);
    bp.update(&b);

    auto candidates = bp.getCandidates(&a);

    assert(candidates.size() == 1 &&
        "Touching shapes must be returned as candidates");

    assert(containsCtx(candidates, "B") &&
        "Touching shape B must be returned");
}

void testSmallVsLargeShape2D() {
    Broadphase2D bp(1.0f);

    Circle small({0, 5}, 0.0001f);
    Box2D large;
    large.center = {0, 0};
    large.halfExtents = {5, 5};
    large.axes[0] = {1, 0};
    large.axes[1] = {0, 1};

    small.ctx = (void*)"Small";
    large.ctx = (void*)"Large";

    bp.update(&small);
    bp.update(&large);

    auto candidates = bp.getCandidates(&small);

    assert(candidates.size() == 1 &&
        "Small shape inside large shape must produce one candidate");

    assert(containsCtx(candidates, "Large") &&
        "Large shape must be candidate for small shape");
}

void testRayMaxDistance2D() {
    Broadphase2D bp(1.0f);

    Box2D inside;
    inside.center = {6.0f, 0};
    inside.halfExtents = {2.0f, 1.0f};
    inside.axes[0] = {1, 0};
    inside.axes[1] = {0, 1};

    Box2D touching;
    touching.center = {6.0f, 0};
    touching.halfExtents = {1.0f, 1.0f};
    touching.axes[0] = {1, 0};
    touching.axes[1] = {0, 1};

    Box2D outside;
    outside.center = {7.5f, 0};
    outside.halfExtents = {1.0f, 1.0f};
    outside.axes[0] = {1, 0};
    outside.axes[1] = {0, 1};

    inside.ctx = (void*)"Inside";
    touching.ctx = (void*)"Touching";
    outside.ctx = (void*)"Outside";

    bp.update(&inside);
    bp.update(&touching);
    bp.update(&outside);

    ray::Ray2D ray;
    ray.origin = {0, 0};
    ray.direction = {1, 0}; // normalized

    float maxDistance = 5.0f;

    auto candidates = bp.getCandidates(ray, maxDistance);

    assert(containsCtx(candidates, "Inside") &&
        "Shape intersecting ray within maxDistance must be included");

    assert(containsCtx(candidates, "Touching") &&
        "Shape touching maxDistance boundary must be included");

    assert(!containsCtx(candidates, "Outside") &&
        "Shape fully beyond maxDistance must not be included");
}

int main() {
    testShapeCandidatesBasic2D();
    testTouchingShapes2D();
    testSmallVsLargeShape2D();
    testRayMaxDistance2D();

    std::cout << "[broadphase2D] All tests passed.\n";
    return 0;
}
