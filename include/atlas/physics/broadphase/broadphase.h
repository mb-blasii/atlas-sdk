#pragma once

#include <vector>
#include <atlas/core/vectors/vec3i.h>
#include <atlas/physics/shapes/shape.h>
#include <atlas/physics/raycast/raycast.h>

namespace atlas::physics::bp {

    struct Cell {
        std::vector<shape::Shape*> shapes; //shapes touching this cell
    };

    struct Broadphase {
    public:
        const float cellSize; // size of the grid cell. Should be based on max average extents of shapes.
        const float scaleFactor; // factor applied to shape AABB for broadphase margin

        //Constructor
        explicit Broadphase(float cellSize, float scaleFactor = 1.1f);

        void update(shape::Shape* s); //Update or insert a single shape
        void update(shape::Shape** shapes, size_t length); //Batch update multiple shapes
        void updateAll(); //Clear the grid and recalculate all ssaved shapes
        void remove(shape::Shape* s);
        bool contains(shape::Shape* s) const;

        //Returns potential collision candidates for a shape (AABB filtered)
        std::vector<shape::Shape*> getCandidates(const shape::Shape* queryShape) const;

        //Returns potential raycast candidates (AABB filtered)
        std::vector<shape::Shape*> getCandidates(const ray::Ray& ray, float maxDistance) const;

    private:
        std::unordered_map<core::vec::Vec3i, Cell> grid;
        std::unordered_map<shape::Shape*, shape::AABB> shapeBounds;

        //Utility functions
        core::vec::Vec3i positionToCell(const core::vec::Vec3& pos) const;  //Convert world position to cell coordinates
        std::vector<core::vec::Vec3i> getOccupiedCells(const shape::AABB& aabb) const; //Get all cells occupied by an AABB
        std::vector<core::vec::Vec3i> getRayCells(const ray::Ray& ray, float maxDistance) const; //Get all cells passed by the ray
    };

}
