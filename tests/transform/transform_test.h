#pragma once

void testTransformLocalWorld();
void testTransformParentChildTranslation();
void testTransformParentRotation();
void testTransformReparentPreserveWorld();
void testTransformHierarchyStructure();
void testTransformHierarchyReorder();
void testTransformDirections();
void testTransformInverseOperations();

void testTranslateLocal_NoParent();
void testTranslateLocal_WithRotation();
void testTranslateWorld_NoParent();
void testTranslateWorld_WithRotatedParent();

void testRotateLocal_NoParent();
void testRotateLocal_Cumulative();
void testRotateWorld_NoParent();
void testRotateWorld_WithRotatedParent();
