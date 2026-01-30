#include "transform_test.h"

#include <atlas/core/transform/transform.h>
#include <atlas/core/math/math.h>

#include <cassert>
#include <iostream>
#include <cstring>

using namespace atlas::core::transform;
using namespace atlas::core::vec;
using namespace atlas::core::quat;
using namespace atlas::core::math;

// Helpers

static const char* nameOf(const Transform* t) {
    return static_cast<const char*>(t->ctx);
}

void assertVec3Equal(const Vec3& a, const Vec3& b, const char* msg = "") {
    assert(nearlyEqual(a.x, b.x) && msg);
    assert(nearlyEqual(a.y, b.y) && msg);
    assert(nearlyEqual(a.z, b.z) && msg);
}

void assertChildNamed(
    const Transform& parent,
    std::size_t index,
    const char* expected,
    const char* msg
) {
    Transform* child = parent.getChild(index);
    assert(child && "Child should exist");
    assert(std::strcmp(nameOf(child), expected) == 0 && msg);
}

// Tests

void testTransformLocalWorld() {
    Transform t;
    t.ctx = (void*)"root";

    t.setLocalPosition(Vec3{1, 2, 3});
    t.setLocalScale(Vec3{2, 2, 2});

    assertVec3Equal(
        t.getWorldPosition(),
        Vec3{1, 2, 3},
        "[root] World position should equal local position"
    );

    assertVec3Equal(
        t.getWorldScale(),
        Vec3{2, 2, 2},
        "[root] World scale should equal local scale"
    );
}

void testTransformParentChildTranslation() {
    Transform parent;
    Transform child;

    parent.ctx = (void*)"parent";
    child.ctx  = (void*)"child";

    parent.setLocalPosition(Vec3{10, 0, 0});

    child.setParent(&parent);
    child.setLocalPosition(Vec3{1, 0, 0});

    assertVec3Equal(
        child.getWorldPosition(),
        Vec3{11, 0, 0},
        "[child] World position should be parent + local"
    );

    parent.setLocalPosition(Vec3{20, 0, 0});

    assertVec3Equal(
        child.getWorldPosition(),
        Vec3{21, 0, 0},
        "[child] World position should update after parent move"
    );
}

void testTransformParentRotation() {
    Transform parent;
    Transform child;

    parent.ctx = (void*)"parent";
    child.ctx  = (void*)"child";

    parent.setLocalRotation(
        fromEuler(Vec3{0, degToRad(180.0f), 0})
    );

    child.setParent(&parent);
    child.setLocalPosition(Vec3{0, 0, 1});

    assertVec3Equal(
        child.getWorldPosition(),
        Vec3{0, 0, -1},
        "[child] Child should rotate around parent origin"
    );
}

void testTransformReparentPreserveWorld() {
    Transform parentA;
    Transform parentB;
    Transform child;

    parentA.ctx = (void*)"parentA";
    parentB.ctx = (void*)"parentB";
    child.ctx   = (void*)"child";

    parentA.setLocalPosition(Vec3{10, 0, 0});
    parentB.setLocalPosition(Vec3{-5, 0, 0});

    child.setParent(&parentA);
    child.setLocalPosition(Vec3{1, 0, 0});

    Vec3 worldBefore = child.getWorldPosition();

    child.setParent(&parentB);

    assertVec3Equal(
        child.getWorldPosition(),
        worldBefore,
        "[child] Reparenting should preserve world position"
    );
}

void testTransformHierarchyStructure() {
    Transform root;
    Transform a, b, c;

    root.ctx = (void*)"root";
    a.ctx    = (void*)"A";
    b.ctx    = (void*)"B";
    c.ctx    = (void*)"C";

    a.setParent(&root);
    b.setParent(&root);
    c.setParent(&root);

    assert(root.getChildCount() == 3 && "[root] Should have 3 children");

    assertChildNamed(root, 0, "A", "[root] First child should be A");
    assertChildNamed(root, 1, "B", "[root] Second child should be B");
    assertChildNamed(root, 2, "C", "[root] Third child should be C");
}

void testTransformHierarchyReorder() {
    Transform root;
    Transform a, b, c;

    root.ctx = (void*)"root";
    a.ctx    = (void*)"A";
    b.ctx    = (void*)"B";
    c.ctx    = (void*)"C";

    a.setParent(&root);
    b.setParent(&root);
    c.setParent(&root);

    root.reorderChild(0, 2);

    assertChildNamed(root, 0, "B", "[root] First child should be B after reorder");
    assertChildNamed(root, 1, "C", "[root] Second child should be C after reorder");
    assertChildNamed(root, 2, "A", "[root] Third child should be A after reorder");
}

void testTransformDirections() {
    Transform t;
    t.ctx = (void*)"dir";

    t.setLocalRotation(fromEuler(Vec3{0, degToRad(90.0f), 0}));

    assertVec3Equal(
        t.forward(),
        Vec3{1, 0, 0},
        "[dir] Forward should rotate with transform"
    );

    assertVec3Equal(
        t.up(),
        Vec3{0, 1, 0},
        "[dir] Up direction should remain unchanged"
    );
}

void testTransformInverseOperations() {
    Transform t;
    t.ctx = (void*)"inverse";

    t.setLocalPosition(Vec3{5, 0, 0});
    t.setLocalRotation(fromEuler(Vec3{0, degToRad(180.0f), 0}));

    Vec3 local{0, 0, 1};
    Vec3 world = t.transformPoint(local);
    Vec3 back  = t.inverseTransformPoint(world);

    assertVec3Equal(
        back,
        local,
        "[inverse] Inverse transform should restore local point"
    );
}

void testTranslateLocal_NoParent() {
    Transform t;
    t.ctx = (void*)"local_no_parent";

    t.setLocalPosition(Vec3{0, 0, 0});
    t.translateLocal(Vec3{1, 2, 3});

    assertVec3Equal(
        t.getWorldPosition(),
        Vec3{1, 2, 3},
        "[translateLocal] Local translation without parent should move world position equally"
    );
}

void testTranslateLocal_WithRotation() {
    Transform t;
    t.ctx = (void*)"local_rotated";

    t.setLocalRotation(fromEuler(Vec3{0, degToRad(90.0f), 0}));
    t.setLocalPosition(Vec3{0, 0, 0});

    t.translateLocal(Vec3{0, 0, 1});

    assertVec3Equal(
        t.getWorldPosition(),
        Vec3{1, 0, 0},
        "[translateLocal] Local translation should follow rotated local axes"
    );
}

void testTranslateWorld_NoParent() {
    Transform t;
    t.ctx = (void*)"world_no_parent";

    t.setLocalRotation(fromEuler(Vec3{0, degToRad(90.0f), 0}));
    t.setLocalPosition(Vec3{0, 0, 0});

    t.translateWorld(Vec3{0, 0, 1});

    assertVec3Equal(
        t.getWorldPosition(),
        Vec3{0, 0, 1},
        "[translateWorld] World translation should ignore object rotation"
    );
}

void testTranslateWorld_WithRotatedParent() {
    Transform parent;
    Transform child;

    parent.ctx = (void*)"parent";
    child.ctx  = (void*)"child";

    parent.setLocalRotation(fromEuler(Vec3{0, degToRad(90.0f), 0}));
    parent.setLocalPosition(Vec3{0, 0, 0});

    child.setParent(&parent);
    child.setLocalPosition(Vec3{5, 0, 0});

    assertVec3Equal(
        child.getWorldPosition(),
        Vec3{0, 0, -5},
        "[translateWorld] Child world position should reflect parent rotation"
    );

    child.translateWorld(Vec3{5, 0, 1});

    assertVec3Equal(
        child.getWorldPosition(),
        Vec3{5, 0, -4},
        "[translateWorld] World translation with rotated parent should move along global axes"
    );
}


int main() {
    testTransformLocalWorld();
    testTransformParentChildTranslation();
    testTransformParentRotation();
    testTransformReparentPreserveWorld();
    testTransformHierarchyStructure();
    testTransformHierarchyReorder();
    testTransformDirections();
    testTransformInverseOperations();

    testTranslateLocal_NoParent();
    testTranslateLocal_WithRotation();
    testTranslateWorld_NoParent();
    testTranslateWorld_WithRotatedParent();

    std::cout << "[transform] All tests passed successfully.\n";
    return 0;
}
