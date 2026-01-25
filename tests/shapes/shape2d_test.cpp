#include <iostream>
#include <cassert>

#include "shape2d_test.h"
#include <atlas/physics/shapes/shape2d.h>

using namespace atlas::physics::shape;
using atlas::core::vec::Vec2;

// Point-Circle
void testPointCircle() {
    Circle c{ Vec2{0,0}, 1.0f };

    // 1. Point outside
    {
        Vec2 p{2,0};
        assert(!overlap(p, c) && "Point outside should not overlap");
    }

    // 2. Point inside
    {
        Vec2 p{0.5f,0};
        assert(overlap(p, c) && "Point inside should overlap");
    }

    // 3. Point on boundary
    {
        Vec2 p{1,0};
        assert(overlap(p, c) && "Point on boundary should overlap");
    }
}

// Point-Rect
void testPointRect() {
    Rect r{ Vec2{0,0}, Vec2{1,1} };

    // 1. Point outside
    {
        Vec2 p{2,0};
        assert(!overlap(p, r) && "Point outside should not overlap");
    }

    // 2. Point inside
    {
        Vec2 p{0.5f,0.5f};
        assert(overlap(p, r) && "Point inside should overlap");
    }

    // 3. Point on boundary
    {
        Vec2 p{1,0};
        assert(overlap(p, r) && "Point on boundary should overlap");
    }
}

// Point-Capsule2D
void testPointCapsule() {
    Capsule2D cap{ Vec2{0,-1}, Vec2{0,1}, 0.5f };

    // 1. Point outside
    {
        Vec2 p{1,0};
        assert(!overlap(p,cap) && "Point outside should not overlap");
    }

    // 2. Point inside
    {
        Vec2 p{0.2f,0};
        assert(overlap(p,cap) && "Point inside should overlap");
    }

    // 3. Point on boundary
    {
        Vec2 p{0.5f,1};
        assert(overlap(p,cap) && "Point on boundary should overlap");
    }
}

// RECT
void testRectRect()
{
    Rect a{ {0.0f, 0.0f}, {1.0f, 1.0f} };
    Rect b{ {1.5f, 0.0f}, {1.0f, 1.0f} };
    Rect c{ {3.5f, 0.0f}, {1.0f, 1.0f} };

    assert(overlap(a, b) && "Rect-Rect should overlap");
    assert(!overlap(a, c) && "Rect-Rect should NOT overlap");
}

// CIRCLE
void testCircleCircle()
{
    Circle a{ {0.0f, 0.0f}, 1.0f };
    Circle b{ {1.5f, 0.0f}, 1.0f };
    Circle c{ {3.0f, 0.0f}, 1.0f };

    assert(overlap(a, b) && "Circle-Circle should overlap");
    assert(!overlap(a, c) && "Circle-Circle should NOT overlap");
}

// RECT / CIRCLE
void testRectCircle()
{
    Rect r{ {0.0f, 0.0f}, {1.0f, 1.0f} };
    Circle c1{ {0.5f, 0.5f}, 0.5f };
    Circle c2{ {3.0f, 3.0f}, 0.5f };

    assert(overlap(r, c1) && "Rect-Circle should overlap");
    assert(!overlap(r, c2) && "Rect-Circle should NOT overlap");
}

// CAPSULE / CIRCLE
void testCapsuleCircle()
{
    Capsule2D cap{
        { -1.0f, 0.0f },
        {  1.0f, 0.0f },
        0.5f
    };

    Circle c1{ {0.0f, 0.2f}, 0.3f };
    Circle c2{ {0.0f, 2.0f}, 0.3f };

    assert(overlap(cap, c1) && "Capsule-Circle should overlap");
    assert(!overlap(cap, c2) && "Capsule-Circle should NOT overlap");
}

// CAPSULE / RECT
void testCapsuleRect()
{
    Capsule2D cap{
        { -1.0f, 0.0f },
        {  1.0f, 0.0f },
        0.5f
    };

    Rect r1{ {0.0f, 0.0f}, {0.5f, 0.5f} };
    Rect r2{ {0.0f, 2.0f}, {0.5f, 0.5f} };

    assert(overlap(cap, r1) && "Capsule-Rect should overlap");
    assert(!overlap(cap, r2) && "Capsule-Rect should NOT overlap");
}

// CAPSULE / CAPSULE
void testCapsuleCapsule()
{
    Capsule2D a{
        { -1.0f, 0.0f },
        {  1.0f, 0.0f },
        0.5f
    };

    Capsule2D b{
        { -1.0f, 0.4f },
        {  1.0f, 0.4f },
        0.5f
    };

    Capsule2D c{
        { -1.0f, 3.0f },
        {  1.0f, 3.0f },
        0.5f
    };

    assert(overlap(a, b) && "Capsule-Capsule should overlap");
    assert(!overlap(a, c) && "Capsule-Capsule should NOT overlap");
}

void testBoxRect()
{
    //Square with l=1 rotated 45 degrees on the origin
    //Rect is testing one of the corners that would be occupied by the Box's Rect

    Box2D box;
    box.center = Vec2{ 0.0f, 0.0f };
    box.halfExtents = Vec2{ 1.0f, 1.0f };

    const float invSqrt2 = 0.7071067811865475f;
    box.axes[0] = Vec2{  invSqrt2,  invSqrt2 };
    box.axes[1] = Vec2{ -invSqrt2,  invSqrt2 };

    Rect rect;
    rect.center = Vec2{ 1.35f, 0.30f };
    rect.halfExtents = Vec2{ 0.1f, 0.1f };

    assert(!overlap(rect, box) && "Rect is inside Box AABB but outside real Box");

    Rect boxRect = box.computeRect();
    assert(overlap(boxRect, rect) && "Rect must overlap Box bounding Rect");

    Rect rect2;
    rect2.center = Vec2{ 1.35f, 0.30f };
    rect2.halfExtents = Vec2{ 0.5f, 0.1f };
    assert(overlap(rect2, box) && "Rect extents should overlap with the Box");
}

void testBoxBox()
{
    const float invSqrt2 = 0.7071067811865475f;

    // Box A (45°)
    Box2D boxA;
    boxA.center = Vec2{ 0.0f, 0.0f };
    boxA.halfExtents = Vec2{ 1.0f, 0.5f };
    boxA.axes[0] = Vec2{  invSqrt2,  invSqrt2 };
    boxA.axes[1] = Vec2{ -invSqrt2,  invSqrt2 };

    // Box B (-30°)
    Box2D boxB;
    boxB.halfExtents = Vec2{ 0.6f, 0.4f };

    const float cos30 = 0.8660254037844386f;
    const float sin30 = 0.5f;

    boxB.axes[0] = Vec2{  cos30, -sin30 };
    boxB.axes[1] = Vec2{  sin30,  cos30 };

    // Distance chosen so that:
    // - projection on all SAT axes is strictly separated
    // - no center lies inside the other box
    boxB.center = Vec2{ 2.6f, 0.0f };

    assert(!overlap(boxA, boxB) &&
           "Box2D should NOT overlap (different rotations, clear separation)");

    // Move boxB closer along X
    boxB.center = Vec2{ 1.54935f, 0.0f };

    assert(overlap(boxA, boxB) &&
           "Box2D should overlap after reducing separation");
}

void testBoxCapsule()
{
    const float invSqrt2 = 0.7071067811865475f;

    // Box rotated 45°
    Box2D box;
    box.center = Vec2{ 0.0f, 0.0f };
    box.halfExtents = Vec2{ 1.0f, 1.0f };
    box.axes[0] = Vec2{  invSqrt2,  invSqrt2 };
    box.axes[1] = Vec2{ -invSqrt2,  invSqrt2 };

    Capsule2D capsule;
    capsule.radius = 0.15f;

    // Capsule completely outside (NO overlap)
    // Degenerate capsule (point + radius)
    // Distance from OBB corner is strictly > radius
    capsule.a = Vec2{ 1.80f, 0.0f };
    capsule.b = capsule.a;

    assert(!overlap(box, capsule) &&
           "Capsule should NOT overlap Box2D");

    // Capsule overlapping
    capsule.a = Vec2{ 1.5f, 0.0f }; //Overlap on very edge point
    capsule.b = capsule.a;

    assert(overlap(box, capsule) &&
           "Capsule should overlap Box2D");
}

int main()
{
    testPointCircle();
    testPointRect();
    testPointCapsule();
    testRectRect();
    testCircleCircle();
    testRectCircle();
    testCapsuleCircle();
    testCapsuleRect();
    testCapsuleCapsule();

    testBoxRect();
    testBoxBox();
    testBoxCapsule();

    std::cout << "[shapes2] All tests passed.\n";
    return 0;
}
