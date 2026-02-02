#include <atlas/physics/broadphase/broadphase2d.h>

#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <ranges>
#include <limits>

namespace atlas::physics::bp {

    Broadphase2D::Broadphase2D(float cellSize, float scaleFactor)
        : cellSize(cellSize), scaleFactor(scaleFactor) {
    }

#pragma region utility functions

    core::vec::Vec2i Broadphase2D::positionToCell(const core::vec::Vec2& pos) const {
        return core::vec::Vec2i{
            static_cast<int>(std::floor(pos.x / cellSize)),
            static_cast<int>(std::floor(pos.y / cellSize))
        };
    }

    std::vector<core::vec::Vec2i> Broadphase2D::getOccupiedCells(const shape::Rect& rect) const {
        core::vec::Vec2 min = rect.center - rect.halfExtents;
        core::vec::Vec2 max = rect.center + rect.halfExtents;

        core::vec::Vec2i minCell = positionToCell(min);
        core::vec::Vec2i maxCell = positionToCell(max);

        std::vector<core::vec::Vec2i> cells;
        for (int x = minCell.x; x <= maxCell.x; ++x)
            for (int y = minCell.y; y <= maxCell.y; ++y)
                cells.emplace_back(core::vec::Vec2i{x, y});

        return cells;
    }

    std::vector<core::vec::Vec2i> Broadphase2D::getRayCells(const ray::Ray2D& ray, float maxDistance) const {
        std::vector<core::vec::Vec2i> cells;
        cells.reserve(16);

        const core::vec::Vec2& origin = ray.origin;
        const core::vec::Vec2& dir = ray.direction;

        core::vec::Vec2i cell = positionToCell(origin);

        core::vec::Vec2i step;
        core::vec::Vec2 tMax;
        core::vec::Vec2 tDelta;

        auto initAxis = [&](float originCoord, float dirCoord, int cellCoord,
                            int& stepOut, float& tMaxOut, float& tDeltaOut)
        {
            if (dirCoord > 0.0f) {
                stepOut = 1;
                float next = (cellCoord + 1) * cellSize;
                tMaxOut = (next - originCoord) / dirCoord;
                tDeltaOut = cellSize / dirCoord;
            }
            else if (dirCoord < 0.0f) {
                stepOut = -1;
                float next = cellCoord * cellSize;
                tMaxOut = (next - originCoord) / dirCoord;
                tDeltaOut = -cellSize / dirCoord;
            }
            else {
                stepOut = 0;
                tMaxOut = std::numeric_limits<float>::infinity();
                tDeltaOut = std::numeric_limits<float>::infinity();
            }
        };

        initAxis(origin.x, dir.x, cell.x, step.x, tMax.x, tDelta.x);
        initAxis(origin.y, dir.y, cell.y, step.y, tMax.y, tDelta.y);

        float t = 0.0f;

        while (t <= maxDistance) {
            cells.push_back(cell);

            if (tMax.x < tMax.y) {
                cell.x += step.x;
                t = tMax.x;
                tMax.x += tDelta.x;
            } else {
                cell.y += step.y;
                t = tMax.y;
                tMax.y += tDelta.y;
            }
        }

        return cells;
    }

#pragma endregion

#pragma region broadphase grid functions

    bool Broadphase2D::contains(shape::Shape2D* s) const {
        return shapeBounds.contains(s);
    }

    void Broadphase2D::remove(shape::Shape2D* s) {
        auto it = shapeBounds.find(s);
        if (it == shapeBounds.end()) return;

        std::vector<core::vec::Vec2i> oldCells = getOccupiedCells(it->second);
        for (auto& c : oldCells) {
            auto gridIt = grid.find(c);
            if (gridIt != grid.end()) {
                auto& shapes = gridIt->second.shapes;
                std::erase(shapes, s);
                if (shapes.empty())
                    grid.erase(gridIt);
            }
        }

        shapeBounds.erase(it);
    }

    void Broadphase2D::update(shape::Shape2D* s) {
        shape::Rect rect = s->computeRect(scaleFactor);

        auto it = shapeBounds.find(s);
        if (it != shapeBounds.end()) {
            std::vector<core::vec::Vec2i> oldCells = getOccupiedCells(it->second);
            std::vector<core::vec::Vec2i> newCells = getOccupiedCells(rect);

            std::unordered_set<core::vec::Vec2i> newSet(newCells.begin(), newCells.end());

            for (auto& c : oldCells) {
                if (!newSet.contains(c)) {
                    auto gridIt = grid.find(c);
                    if (gridIt != grid.end()) {
                        auto& shapes = gridIt->second.shapes;
                        std::erase(shapes, s);
                        if (shapes.empty())
                            grid.erase(gridIt);
                    }
                }
            }

            for (auto& c : newCells) {
                auto& shapes = grid[c].shapes;
                if (std::ranges::find(shapes, s) == shapes.end())
                    shapes.push_back(s);
            }

            it->second = rect;
        }
        else {
            std::vector<core::vec::Vec2i> cells = getOccupiedCells(rect);
            for (auto& c : cells)
                grid[c].shapes.push_back(s);

            shapeBounds[s] = rect;
        }
    }

    void Broadphase2D::update(shape::Shape2D** shapes, size_t length) {
        for (size_t i = 0; i < length; ++i)
            update(shapes[i]);
    }

    void Broadphase2D::updateAll() {
        grid.clear();
        for (const auto& s : shapeBounds | std::views::keys) {
            shape::Rect rect = s->computeRect(scaleFactor);
            std::vector<core::vec::Vec2i> cells = getOccupiedCells(rect);
            for (auto& c : cells)
                grid[c].shapes.push_back(s);

            shapeBounds[s] = rect;
        }
    }

#pragma endregion

#pragma region getCandidates

    std::vector<shape::Shape2D*> Broadphase2D::getCandidates(const shape::Shape2D* queryShape) const {
        std::vector<shape::Shape2D*> result;
        std::unordered_set<shape::Shape2D*> unique;

        shape::Rect queryRect = queryShape->computeRect(scaleFactor);
        std::vector<core::vec::Vec2i> cells = getOccupiedCells(queryRect);

        for (const auto& cell : cells) {
            auto it = grid.find(cell);
            if (it == grid.end()) continue;

            for (shape::Shape2D* s : it->second.shapes) {
                if (s == queryShape || unique.contains(s)) continue;

                shape::Rect realQuery = queryShape->computeRect(1.0f);
                shape::Rect realRect = s->computeRect(1.0f);

                if (shape::overlap(realQuery, realRect)) {
                    unique.insert(s);
                    result.push_back(s);
                }
            }
        }

        return result;
    }

    std::vector<shape::Shape2D*> Broadphase2D::getCandidates(const ray::Ray2D& ray, float maxDistance) const {
        std::vector<shape::Shape2D*> result;
        std::unordered_set<shape::Shape2D*> unique;

        std::vector<core::vec::Vec2i> cells = getRayCells(ray, maxDistance);

        for (const auto& cell : cells) {
            auto it = grid.find(cell);
            if (it == grid.end()) continue;

            for (shape::Shape2D* s : it->second.shapes) {
                if (unique.contains(s)) continue;

                shape::Rect realRect = s->computeRect(1.0f);
                ray::RayResult2D out;

                if (ray::raycast(ray, realRect, out) && out.distance <= maxDistance) {
                    unique.insert(s);
                    result.push_back(s);
                }
            }
        }

        return result;
    }

#pragma endregion
}
