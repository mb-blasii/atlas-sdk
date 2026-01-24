#include <atlas/physics/broadphase/broadphase.h>
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <ranges>

namespace atlas::physics::bp {
    Broadphase::Broadphase(float cellSize, float scaleFactor)
        : cellSize(cellSize), scaleFactor(scaleFactor) {
    }

#pragma region utility functions

    core::vec::Vec3i Broadphase::positionToCell(const core::vec::Vec3 &pos) const {
        return core::vec::Vec3i{
            static_cast<int>(std::floor(pos.x / cellSize)),
            static_cast<int>(std::floor(pos.y / cellSize)),
            static_cast<int>(std::floor(pos.z / cellSize))
        };
    }

    std::vector<core::vec::Vec3i> Broadphase::getOccupiedCells(const shape::AABB &aabb) const {
        core::vec::Vec3 min = aabb.center - aabb.halfExtents;
        core::vec::Vec3 max = aabb.center + aabb.halfExtents;

        core::vec::Vec3i minCell = positionToCell(min);
        core::vec::Vec3i maxCell = positionToCell(max);

        std::vector<core::vec::Vec3i> cells;
        for (int x = minCell.x; x <= maxCell.x; ++x)
            for (int y = minCell.y; y <= maxCell.y; ++y)
                for (int z = minCell.z; z <= maxCell.z; ++z)
                    cells.emplace_back(core::vec::Vec3i{x, y, z});

        return cells;
    }

    std::vector<core::vec::Vec3i> Broadphase::getRayCells(const ray::Ray &ray, float maxDistance) const {
        std::vector<core::vec::Vec3i> cells;
        cells.reserve(32); // avoid reallocs

        const core::vec::Vec3 &origin = ray.origin;
        const core::vec::Vec3 &dir = ray.direction;

        core::vec::Vec3i cell = positionToCell(origin);

        core::vec::Vec3i step;
        core::vec::Vec3 tMax;
        core::vec::Vec3 tDelta;

        auto initAxis = [&](float originCoord, float dirCoord, int cellCoord,
                            int &stepOut, float &tMaxOut, float &tDeltaOut) {
            if (dirCoord > 0.0f) {
                stepOut = 1;
                float next = (cellCoord + 1) * cellSize;
                tMaxOut = (next - originCoord) / dirCoord;
                tDeltaOut = cellSize / dirCoord;
            } else if (dirCoord < 0.0f) {
                stepOut = -1;
                float next = cellCoord * cellSize;
                tMaxOut = (next - originCoord) / dirCoord;
                tDeltaOut = -cellSize / dirCoord;
            } else {
                stepOut = 0;
                tMaxOut = std::numeric_limits<float>::infinity();
                tDeltaOut = std::numeric_limits<float>::infinity();
            }
        };

        initAxis(origin.x, dir.x, cell.x, step.x, tMax.x, tDelta.x);
        initAxis(origin.y, dir.y, cell.y, step.y, tMax.y, tDelta.y);
        initAxis(origin.z, dir.z, cell.z, step.z, tMax.z, tDelta.z);

        float t = 0.0f;

        while (t <= maxDistance) {
            cells.push_back(cell);

            if (tMax.x < tMax.y) {
                if (tMax.x < tMax.z) {
                    cell.x += step.x;
                    t = tMax.x;
                    tMax.x += tDelta.x;
                } else {
                    cell.z += step.z;
                    t = tMax.z;
                    tMax.z += tDelta.z;
                }
            } else {
                if (tMax.y < tMax.z) {
                    cell.y += step.y;
                    t = tMax.y;
                    tMax.y += tDelta.y;
                } else {
                    cell.z += step.z;
                    t = tMax.z;
                    tMax.z += tDelta.z;
                }
            }
        }

        return cells;
    }

#pragma endregion

#pragma region broadphase grid functions

    bool Broadphase::contains(shape::Shape *s) const {
        return shapeBounds.contains(s);
    }

    void Broadphase::remove(shape::Shape *s) {
        auto it = shapeBounds.find(s);
        if (it == shapeBounds.end()) return;

        std::vector<core::vec::Vec3i> oldCells = getOccupiedCells(it->second);
        for (auto &c: oldCells) {
            auto gridIt = grid.find(c);
            if (gridIt != grid.end()) {
                auto &shapes = gridIt->second.shapes;
                std::erase(shapes, s);
                if (shapes.empty()) grid.erase(gridIt); // remove empty cell
            }
        }

        shapeBounds.erase(it);
    }

    void Broadphase::update(shape::Shape *s) {
        shape::AABB aabb = s->computeAABB(scaleFactor);

        auto it = shapeBounds.find(s);
        if (it != shapeBounds.end()) {
            //Shape exists -> update
            std::vector<core::vec::Vec3i> oldCells = getOccupiedCells(it->second);
            std::vector<core::vec::Vec3i> newCells = getOccupiedCells(aabb);

            std::unordered_set newSet(newCells.begin(), newCells.end());

            //Remove from cells no longer occupied
            for (auto &c: oldCells) {
                if (!newSet.contains(c)) {
                    auto gridIt = grid.find(c);
                    if (gridIt != grid.end()) {
                        auto &shapes = gridIt->second.shapes;
                        std::erase(shapes, s);
                        if (shapes.empty()) grid.erase(gridIt);
                    }
                }
            }

            //Insert into new cells
            for (auto &c: newCells) {
                auto &[shapes] = grid[c]; // creates if not exists
                if (std::ranges::find(shapes, s) == shapes.end())
                    shapes.push_back(s);
            }

            it->second = aabb;
        } else {
            //New shape -> insert
            std::vector<core::vec::Vec3i> cells = getOccupiedCells(aabb);
            for (auto &c: cells) grid[c].shapes.push_back(s);
            shapeBounds[s] = aabb;
        }
    }

    void Broadphase::update(shape::Shape **shapes, size_t length) {
        for (size_t i = 0; i < length; ++i)
            update(shapes[i]);
    }

    void Broadphase::updateAll() {
        grid.clear();
        for (const auto &s: shapeBounds | std::views::keys) {
            shape::AABB aabb = s->computeAABB(scaleFactor);
            std::vector<core::vec::Vec3i> cells = getOccupiedCells(aabb);
            for (auto &c: cells) grid[c].shapes.push_back(s);
            shapeBounds[s] = aabb;
        }
    }

#pragma endregion

#pragma region getCandidates

    std::vector<shape::Shape *> Broadphase::getCandidates(const shape::Shape *queryShape) const {
        std::vector<shape::Shape *> result;

        //Scaled AABB for cells query
        shape::AABB queryAABB = queryShape->computeAABB(scaleFactor);
        std::vector<core::vec::Vec3i> cells = getOccupiedCells(queryAABB);

        std::unordered_set<shape::Shape *> unique; //Improve search in O(1)

        for (const auto &cell: cells) {
            auto it = grid.find(cell);
            if (it == grid.end()) continue;

            for (shape::Shape *s: it->second.shapes) {
                if (unique.contains(s) || s == queryShape) continue;

                //Filter using REAL AABB
                shape::AABB filterAABB = queryShape->computeAABB();
                shape::AABB realAABB = s->computeAABB(1.0f);
                if (shape::overlap(filterAABB, realAABB)) {
                    unique.insert(s);
                    result.push_back(s);
                }
            }
        }

        return result;
    }

    std::vector<shape::Shape *> Broadphase::getCandidates(const ray::Ray &ray, float maxDistance) const {
        std::vector<shape::Shape *> result;
        std::unordered_set<shape::Shape *> unique; //Improve search in O(1)

        std::vector<core::vec::Vec3i> cells = getRayCells(ray, maxDistance);

        for (const auto &cell: cells) {
            auto it = grid.find(cell);
            if (it == grid.end()) continue;

            for (shape::Shape *s: it->second.shapes) {
                if (unique.contains(s)) continue;

                //Filter using REAL AABB
                shape::AABB realAABB = s->computeAABB(1.0f);
                ray::RayResult out;
                if (ray::raycast(ray, realAABB, out) && out.distance <= maxDistance) {
                    unique.insert(s);
                    result.push_back(s);
                }
            }
        }

        return result;
    }

#pragma endregion
}
