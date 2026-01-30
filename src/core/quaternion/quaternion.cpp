#include <atlas/core/quaternion/quaternion.h>

#include <cmath>
#include <cassert>
#include <atlas/core/math/constants.h>

namespace atlas::core::quat {

    // Constructors
    Quat::Quat() : x(0), y(0), z(0), w(1) {
    }

    Quat::Quat(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {
    }

    Quat::Quat(const Quat &other) = default;

#pragma region operators

    Quat Quat::operator+(const Quat &rhs) const {
        return Quat(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
    }

    Quat Quat::operator-(const Quat &rhs) const {
        return Quat(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
    }

    Quat Quat::operator*(const Quat &rhs) const {
        return Quat(
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z
        );
    }

    vec::Vec3 Quat::operator*(const vec::Vec3 &rhs) const {
        auto qXYZ = xyz();
        return rhs + vec::cross(qXYZ, vec::cross(qXYZ, rhs) + rhs * w) * 2;
    }

    Quat Quat::operator/(float s) const {
        return Quat(x / s, y / s, z / s, w / s);
    }

    Quat Quat::operator*(float s) const {
        return Quat(x * s, y * s, z * s, w * s);
    }

    Quat& Quat::operator+=(const Quat &rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w += rhs.w;
        return *this;
    }

    Quat& Quat::operator-=(const Quat &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w -= rhs.w;
        return *this;
    }

    // Optional: compound assignment
    Quat& Quat::operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        return *this;
    }

    Quat& Quat::operator*=(const Quat &rhs) {
        *this = *this * rhs;
        return *this;
    }

    Quat& Quat::operator/=(float s) {
        x /= s;
        y /= s;
        z /= s;
        w /= s;
        return *this;
    }

    Quat Quat::operator-() const { return Quat(-x, -y, -z, w); }

    bool Quat::operator==(const Quat &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
    }

    bool Quat::operator!=(const Quat &rhs) const {
        return !(*this == rhs);
    }

#pragma endregion

    // Normalize
    float Quat::length() const { return std::sqrt(x * x + y * y + z * z + w * w); }
    float Quat::lengthSq() const { return x * x + y * y + z * z + w * w; }

    void Quat::normalize() {
        if (float l = length(); l != 0) {
            x /= l;
            y /= l;
            z /= l;
            w /= l;
        }
    }

    Quat Quat::normalized() const {
        Quat q = *this;
        q.normalize();
        return q;
    }

    vec::Vec3 Quat::xyz() const {
        return vec::Vec3{x,y,z};
    }

    #pragma region quaternion functions

    Quat identity() { return Quat(0, 0, 0, 1); }

    // Conjugate & Inverse
    Quat conjugate(const Quat &q) { return Quat(-q.x, -q.y, -q.z, q.w); }

    Quat inverse(const Quat &q) {
        float l2 = q.lengthSq();
        assert(l2 > math::EPS && "Cannot invert zero-length quaternion");
        Quat c = conjugate(q);
        return Quat(c.x / l2, c.y / l2, c.z / l2, c.w / l2);
    }

    // From Euler angles (XYZ order, radians)
    Quat fromEuler(const vec::Vec3 &euler) {
        float cx = std::cos(euler.x * 0.5f), sx = std::sin(euler.x * 0.5f);
        float cy = std::cos(euler.y * 0.5f), sy = std::sin(euler.y * 0.5f);
        float cz = std::cos(euler.z * 0.5f), sz = std::sin(euler.z * 0.5f);

        return Quat(
            sx * cy * cz + cx * sy * sz,
            cx * sy * cz - sx * cy * sz,
            cx * cy * sz + sx * sy * cz,
            cx * cy * cz - sx * sy * sz
        );
    }

    // To Euler angles (XYZ order, radians)
    vec::Vec3 toEuler(const Quat &q) {
        vec::Vec3 euler;
        // roll (X)
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (Y)
        if (float sinp = 2.0f * (q.w * q.y - q.z * q.x); std::abs(sinp) >= 1)
            euler.y = std::copysign(math::HALF_PI, sinp);
        else
            euler.y = std::asin(sinp);

        // yaw (Z)
        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    // Lerp (linear interpolation)
    Quat lerp(const Quat &a, const Quat &b, float t) {
        Quat q = Quat(
            a.x * (1 - t) + b.x * t,
            a.y * (1 - t) + b.y * t,
            a.z * (1 - t) + b.z * t,
            a.w * (1 - t) + b.w * t
        );
        q.normalize();
        return q;
    }

    // Slerp (spherical linear interpolation)
    Quat slerp(const Quat &qa, const Quat &qb, float t) {
        Quat q1 = qa.normalized();
        Quat q2 = qb.normalized();

        float dotProd = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        if (dotProd < 0.0f) {
            q2 = -q2;
            dotProd = -dotProd;
        }

        if (dotProd > 0.9995f) return lerp(q1, q2, t);

        float theta_0 = std::acos(dotProd);
        float theta = theta_0 * t;

        Quat q3 = q2 - q1 * dotProd;
        q3.normalize();

        return q1 * std::cos(theta) + q3 * std::sin(theta);
    }

    // Rotate a vector
    vec::Vec3 rotate(const vec::Vec3 &v, const Quat &q) {
        Quat qv(v.x, v.y, v.z, 0);
        Quat res = q * qv * inverse(q);
        return vec::Vec3(res.x, res.y, res.z);
    }

#pragma endregion

}