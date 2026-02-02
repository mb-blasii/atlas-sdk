#include <atlas/physics/raycast/raycast2d.h>
#include <atlas/core/math/math.h>
#include <cmath>

namespace atlas::physics::ray {
    using namespace core::vec;
    using namespace core::math;
    using namespace shape;

    // Circle
    bool raycast(const Ray2D &ray, const Circle &c, RayResult2D &out) {
        // origin inside shape
        if (overlap(ray.origin, c)) {
            out.hit = true;
            out.point = ray.origin;
            out.normal = Vec2{0, 0};
            out.distance = 0.0f;
            return true;
        }

        Vec2 dir = ray.direction.normalized();
        Vec2 oc = ray.origin - c.center;
        float b = 2.0f * (oc.x * dir.x + oc.y * dir.y);
        float c_val = oc.lengthSq() - c.radius * c.radius;
        float discriminant = b * b - 4.0f * c_val;

        if (discriminant < 0.0f)
            return false;

        float sqrtDisc = std::sqrt(discriminant);
        float t1 = (-b - sqrtDisc) / 2.0f;
        float t2 = (-b + sqrtDisc) / 2.0f;

        float t = (t1 > EPS) ? t1 : ((t2 > EPS) ? t2 : -1.0f);
        if (t < 0.0f) return false;

        out.hit = true;
        out.distance = t;
        out.point = ray.origin + dir * t;
        out.normal = (out.point - c.center).normalized();
        return true;
    }

    // Rect
    bool raycast(const Ray2D &ray, const Rect &r, RayResult2D &out) {
        if (overlap(ray.origin, r)) {
            out.hit = true;
            out.point = ray.origin;
            out.normal = Vec2{0, 0};
            out.distance = 0.0f;
            return true;
        }

        Vec2 dir = ray.direction.normalized();
        Vec2 invDir{1.0f / dir.x, 1.0f / dir.y};
        Vec2 min = r.center - r.halfExtents;
        Vec2 max = r.center + r.halfExtents;

        float t1 = (min.x - ray.origin.x) * invDir.x;
        float t2 = (max.x - ray.origin.x) * invDir.x;
        float t3 = (min.y - ray.origin.y) * invDir.y;
        float t4 = (max.y - ray.origin.y) * invDir.y;

        float tmin = std::fmax(std::fmin(t1, t2), std::fmin(t3, t4));
        float tmax = std::fmin(std::fmax(t1, t2), std::fmax(t3, t4));

        if (tmax < 0.0f || tmin > tmax)
            return false;

        float t = (tmin > EPS) ? tmin : tmax;
        if (t < 0.0f) return false;

        out.hit = true;
        out.distance = t;
        out.point = ray.origin + dir * t;

        // Compute normal
        Vec2 p = out.point;
        if (nearlyEqual(p.x, min.x)) out.normal = Vec2{-1, 0};
        else if (nearlyEqual(p.x, max.x)) out.normal = Vec2{1, 0};
        else if (nearlyEqual(p.y, min.y)) out.normal = Vec2{0, -1};
        else if (nearlyEqual(p.y, max.y)) out.normal = Vec2{0, 1};
        else out.normal = Vec2{0, 0};

        return true;
    }

    // Capsule2D
    bool raycast(const Ray2D &ray, const Capsule2D &cap, RayResult2D &out) {
        if (overlap(ray.origin, cap)) {
            out.hit = true;
            out.point = ray.origin;
            out.normal = Vec2{0, 0};
            out.distance = 0.0f;
            return true;
        }

        // Project ray onto capsule segment
        Vec2 ab = cap.b - cap.a;
        Vec2 ao = ray.origin - cap.a;
        Vec2 d = ray.direction.normalized();

        float abLenSq = ab.lengthSq();
        float tSegment = dot(ao, ab) / abLenSq;
        tSegment = std::fmax(0.0f, std::fmin(1.0f, tSegment));
        Vec2 closest = cap.a + ab * tSegment;

        Vec2 diff = closest - ray.origin;
        float proj = dot(diff, d);
        if (proj < 0.0f)
            return false;

        Vec2 closestPoint = ray.origin + d * proj;
        float distSq = (closest - closestPoint).lengthSq();
        if (distSq > cap.radius * cap.radius)
            return false;

        float offset = std::sqrt(cap.radius * cap.radius - distSq);
        out.distance = proj - offset;
        if (out.distance < EPS) out.distance = 0.0f;

        out.hit = true;
        out.point = ray.origin + d * out.distance;
        out.normal = (out.point - closest).normalized();
        return true;
    }
}
