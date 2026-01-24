#include "broadphase_test.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

#include <atlas/physics/broadphase/broadphase.h>
#include <atlas/physics/shapes/shape.h>
#include <atlas/physics/raycast/raycast.h>
#include <atlas/core/vectors/vec3.h>

using namespace atlas;
using namespace atlas::physics;
using namespace atlas::physics::shape;
using namespace atlas::physics::bp;
using namespace atlas::physics::ray;

static const core::vec::Vec3 AXES[3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

bool containsCtx(const std::vector<Shape *> &shapes, const char *name) {
    for (auto *s: shapes) {
        if (std::strcmp(static_cast<const char *>(s->ctx), name) == 0)
            return true;
    }
    return false;
}


void testShapeCandidatesBasic() {
    Broadphase bp(1.0f);

    Sphere a({0, 0, 0}, 1.0f);
    Sphere b({1.5f, 0, 0}, 1.0f);
    Sphere c({5, 0, 0}, 1.0f);

    a.ctx = (void *) "Shape A";
    b.ctx = (void *) "Shape B";
    c.ctx = (void *) "Shape C";

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

void testTouchingShapes() {
    Broadphase bp(1.0f);

    Sphere a({0, 0, 0}, 1.0f);
    Sphere b({2.0f, 0, 0}, 1.0f); // exactly touching

    a.ctx = (void *) "A";
    b.ctx = (void *) "B";

    bp.update(&a);
    bp.update(&b);

    auto candidates = bp.getCandidates(&a);

    assert(candidates.size() == 1 &&
        "Touching shapes must be returned as candidates");

    assert(containsCtx(candidates, "B") &&
        "Touching shape B must be returned");
}

void testSmallVsLargeShape() {
    Broadphase bp(1.0f);

    Sphere small({0, 5, 0}, 0.0001f);
    Box large({0, 0, 0}, {5, 5, 5}, AXES);

    small.ctx = (void *) "Small";
    large.ctx = (void *) "Large";

    bp.update(&small);
    bp.update(&large);

    auto candidates = bp.getCandidates(&small);

    assert(candidates.size() == 1 &&
        "Small shape inside large shape must produce one candidate");

    assert(containsCtx(candidates, "Large") &&
        "Large shape must be candidate for small shape");
}

void testRayMaxDistance() {
    Broadphase bp(1.0f);

    Box inside(
        {6.0f, 0, 0},
        {2.0f, 1.0f, 1.0f},
        AXES
    );

    Box touching(
        {6.0f, 0, 0},
        {1.0f, 1.0f, 1.0f},
        AXES
    );

    Box outside(
        {7.5f, 0, 0},
        {1.0f, 1.0f, 1.0f},
        AXES
    );

    inside.ctx = (void *) "Inside";
    touching.ctx = (void *) "Touching";
    outside.ctx = (void *) "Outside";

    bp.update(&inside);
    bp.update(&touching);
    bp.update(&outside);

    Ray ray;
    ray.origin = {0, 0, 0};
    ray.direction = {1, 0, 0}; // normalized

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
    testShapeCandidatesBasic();
    testTouchingShapes();
    testSmallVsLargeShape();
    testRayMaxDistance();

    std::cout << "[broadphase] All tests passed.\n";
    return 0;
}