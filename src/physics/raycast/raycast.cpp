#include <atlas/physics/raycast/raycast.h>
#include <atlas/core/math/math.h>

#include <cmath>

namespace atlas::physics::ray {
    using namespace core;
    using namespace vec;

    // Ray vs Sphere
    bool raycast(const Ray &ray, const shape::Sphere &s, RayResult &out) {
        Vec3 dir = ray.direction.normalized();
        Vec3 oc = ray.origin - s.center;

        float a = dot(dir, dir);
        float b = 2.0f * dot(oc, dir);
        float c = dot(oc, oc) - s.radius * s.radius;

        float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f)
            return false;

        float sqrtD = std::sqrt(disc);
        float t0 = (-b - sqrtD) / (2.0f * a);
        float t1 = (-b + sqrtD) / (2.0f * a);

        float t = t0 >= 0.0f ? t0 : t1;
        if (t < 0.0f)
            return false;

        out.hit = true;
        out.distance = t;
        out.point = ray.origin + dir * t;
        out.normal = (out.point - s.center).normalized();
        return true;
    }

    // Ray vs AABB (slab method)
    bool raycast(const Ray &ray, const shape::AABB &b, RayResult &out) {
        Vec3 dir = ray.direction.normalized();

        Vec3 min = b.center - b.halfExtents;
        Vec3 max = b.center + b.halfExtents;

        float tMin = 0.0f;
        float tMax = std::numeric_limits<float>::max();
        Vec3 hitNormal{};

        for (int i = 0; i < 3; ++i) {
            if (math::isZero(std::abs(dir[i]))) {
                if (ray.origin[i] < min[i] || ray.origin[i] > max[i])
                    return false;
            } else {
                float invD = 1.0f / dir[i];
                float t1 = (min[i] - ray.origin[i]) * invD;
                float t2 = (max[i] - ray.origin[i]) * invD;

                float sign = t1 > t2 ? 1.0f : -1.0f;
                if (t1 > t2)
                    std::swap(t1, t2);

                if (t1 > tMin) {
                    tMin = t1;
                    hitNormal = Vec3{};
                    hitNormal[i] = sign;
                }

                tMax = std::min(tMax, t2);
                if (tMin > tMax)
                    return false;
            }
        }

        out.hit = true;
        out.distance = tMin;
        out.point = ray.origin + dir * tMin;
        out.normal = hitNormal;
        return true;
    }

    // Ray vs OBB (ray transformed to local space)
    bool raycast(const Ray &ray, const shape::Box &o, RayResult &out) {
        Vec3 dir = ray.direction.normalized();

        Vec3 p = ray.origin - o.center;
        Vec3 localOrigin{
            dot(p, o.axes[0]),
            dot(p, o.axes[1]),
            dot(p, o.axes[2])
        };

        Vec3 localDir{
            dot(dir, o.axes[0]),
            dot(dir, o.axes[1]),
            dot(dir, o.axes[2])
        };

        shape::AABB localBox;
        localBox.center = Vec3{0.0f, 0.0f, 0.0f};
        localBox.halfExtents = o.halfExtents;

        Ray localRay{localOrigin, localDir};
        RayResult localHit;

        if (!raycast(localRay, localBox, localHit))
            return false;

        out.hit = true;
        out.distance = localHit.distance;
        out.point = ray.origin + dir * localHit.distance;

        out.normal =
                o.axes[0] * localHit.normal.x +
                o.axes[1] * localHit.normal.y +
                o.axes[2] * localHit.normal.z;

        out.normal = out.normal.normalized();
        return true;
    }

    // Helper: Ray vs finite cylinder (capsule body)
    static bool raycastCylinder(
        const Ray &ray,
        const Vec3 &a,
        const Vec3 &b,
        float radius,
        float &outT,
        Vec3 &outNormal) {
        Vec3 d = b - a;
        Vec3 m = ray.origin - a;
        Vec3 n = ray.direction.normalized();

        float dd = dot(d, d);
        float md = dot(m, d);
        float nd = dot(n, d);

        float mn = dot(m, n);
        float nn = dot(n, n);

        float A = dd * nn - nd * nd;
        float B = dd * mn - md * nd;
        float C = dd * dot(m, m) - md * md - radius * radius * dd;

        if (math::isZero(std::abs(A)))
            return false;

        float disc = B * B - A * C;
        if (disc < 0.0f)
            return false;

        float t = (-B - std::sqrt(disc)) / A;
        if (t < 0.0f)
            return false;

        float k = (md + t * nd) / dd;
        if (k < 0.0f || k > 1.0f)
            return false;

        Vec3 hitPoint = ray.origin + n * t;
        Vec3 axisPoint = a + d * k;

        outT = t;
        outNormal = (hitPoint - axisPoint).normalized();
        return true;
    }

    // Ray vs Capsule
    bool raycast(const Ray &ray, const shape::Capsule &c, RayResult &out) {
        // pointInsideCapsule
        if (shape::overlap(ray.origin, c)) {
            out.hit = true;
            out.distance = 0.0f;
            out.point = ray.origin;
            out.normal = Vec3{0, 0, 0}; // default vector for normal, no valid normal can be calculated
            return true;
        }

        Vec3 dir = ray.direction.normalized();

        float closestT = std::numeric_limits<float>::max();
        Vec3 bestNormal{};
        bool hit = false;

        // Cylinder body
        {
            float t;
            Vec3 n;
            if (raycastCylinder(ray, c.a, c.b, c.radius, t, n)) {
                if (t < closestT) {
                    closestT = t;
                    bestNormal = n;
                    hit = true;
                }
            }
        }

        // Sphere A
        {
            shape::Sphere s{c.a, c.radius};
            RayResult r;
            if (raycast(ray, s, r) && r.distance < closestT) {
                closestT = r.distance;
                bestNormal = r.normal;
                hit = true;
            }
        }

        // Sphere B
        {
            shape::Sphere s{c.b, c.radius};
            RayResult r;
            if (raycast(ray, s, r) && r.distance < closestT) {
                closestT = r.distance;
                bestNormal = r.normal;
                hit = true;
            }
        }

        if (!hit)
            return false;

        out.hit = true;
        out.distance = closestT;
        out.point = ray.origin + dir * closestT;
        out.normal = bestNormal;
        return true;
    }
}
