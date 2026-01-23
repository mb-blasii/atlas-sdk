#include <atlas/core/vectors/vec3i.h>

#include <cmath>
#include <cassert>

namespace atlas::core::vec {
    // Constructors
    Vec3i::Vec3i() : x(0), y(0), z(0) {
    }

    Vec3i::Vec3i(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {
    }

    Vec3i::Vec3i(const Vec3i &other) = default;

    // Operators
    Vec3i Vec3i::operator+(const Vec3i &rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    Vec3i Vec3i::operator-(const Vec3i &rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    Vec3i Vec3i::operator*(int scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 Vec3i::operator*(float scalar) const {
        return Vec3(
            static_cast<float>(x) * scalar,
            static_cast<float>(y) * scalar,
            static_cast<float>(z) * scalar
        );
    }

    Vec3 Vec3i::operator/(float scalar) const {
        return Vec3(
            static_cast<float>(x) / scalar,
            static_cast<float>(y) / scalar,
            static_cast<float>(z) / scalar
        );
    }

    Vec3i &Vec3i::operator+=(const Vec3i &rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3i &Vec3i::operator-=(const Vec3i &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3i &Vec3i::operator*=(int scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    bool Vec3i::operator==(const Vec3i &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool Vec3i::operator!=(const Vec3i &rhs) const {
        return !(*this == rhs);
    }

    int &Vec3i::operator[](int i) {
        assert(i >= 0 && i < 3);
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    const int &Vec3i::operator[](int i) const {
        assert(i >= 0 && i < 3);
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    // Functions
    float Vec3i::length() const {
        return std::sqrt(static_cast<float>(x * x + y * y + z * z));
    }

    int64_t Vec3i::lengthSq() const {
        return static_cast<int64_t>(x) * x +
               static_cast<int64_t>(y) * y +
               static_cast<int64_t>(z) * z;
    }

    Vec3 Vec3i::normalized() const {
        float l = length();
        if (l == 0.0f) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }

        return Vec3(
            static_cast<float>(x) / l,
            static_cast<float>(y) / l,
            static_cast<float>(z) / l
        );
    }

#pragma region functions

    int64_t dot(const Vec3i &v1, const Vec3i &v2) {
        return static_cast<int64_t>(v1.x) * v2.x +
               static_cast<int64_t>(v1.y) * v2.y +
               static_cast<int64_t>(v1.z) * v2.z;
    }

    Vec3i cross(const Vec3i &v1, const Vec3i &v2) {
        return Vec3i(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x
        );
    }

    float distance(const Vec3i &v1, const Vec3i &v2) {
        return (v1 - v2).length();
    }

    int64_t distanceSq(const Vec3i &v1, const Vec3i &v2) {
        return (v1 - v2).lengthSq();
    }

    Vec3 lerp(const Vec3i &a, const Vec3i &b, float t) {
        return Vec3(
            static_cast<float>(a.x) + (static_cast<float>(b.x - a.x) * t),
            static_cast<float>(a.y) + (static_cast<float>(b.y - a.y) * t),
            static_cast<float>(a.z) + (static_cast<float>(b.z - a.z) * t)
        );
    }

#pragma endregion
}
