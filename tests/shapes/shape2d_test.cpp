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

    std::cout << "[shapes2] All tests passed.\n";
    return 0;
}
