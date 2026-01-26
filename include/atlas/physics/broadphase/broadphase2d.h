#pragma once

#include <vector>
#include <unordered_map>

#include <atlas/core/vectors/vec2i.h>
#include <atlas/physics/shapes/shape2d.h>
#include <atlas/physics/raycast/raycast2d.h>

namespace atlas::physics::bp {

    struct Cell2D {
        std::vector<shape::Shape2D*> shapes; // Shapes touching this cell
    };

    struct Broadphase2D {
    public:
        const float cellSize;     // Size of a single grid cell
        const float scaleFactor;  // Margin applied to bounding rects

        // Constructor
        explicit Broadphase2D(float cellSize, float scaleFactor = 1.1f);

        // Insert or update a single shape
        void update(shape::Shape2D* s);

        // Batch update
        void update(shape::Shape2D** shapes, size_t length);

        // Rebuild the entire grid
        void updateAll();

        // Remove a shape from the broadphase
        void remove(shape::Shape2D* s);

        // Check if a shape is already tracked
        bool contains(shape::Shape2D* s) const;

        // Returns potential collision candidates for a shape (Rect filtered)
        std::vector<shape::Shape2D*> getCandidates(const shape::Shape2D* queryShape) const;

        // Returns potential raycast candidates (Rect filtered)
        std::vector<shape::Shape2D*> getCandidates(const ray::Ray2D& ray, float maxDistance) const;

    private:
        std::unordered_map<core::vec::Vec2i, Cell2D> grid;
        std::unordered_map<shape::Shape2D*, shape::Rect> shapeBounds;

        // Utility functions

        // Convert world position to cell coordinates
        core::vec::Vec2i positionToCell(const core::vec::Vec2& pos) const;

        // Get all cells occupied by a bounding rect
        std::vector<core::vec::Vec2i> getOccupiedCells(const shape::Rect& rect) const;

        // Get all cells traversed by a ray
        std::vector<core::vec::Vec2i> getRayCells(const ray::Ray2D& ray, float maxDistance) const;
    };

}
