#pragma once
#include <cstdint>
#include <functional>

#include "vec3.h"

namespace atlas::core::vec {
    struct Vec3i {
        int x, y, z;

        // Constructors
        Vec3i();

        Vec3i(int _x, int _y, int _z);

        Vec3i(const Vec3i &other);

        // Operators
        Vec3i operator+(const Vec3i &rhs) const;

        Vec3i operator-(const Vec3i &rhs) const;

        Vec3i operator*(int scalar) const;

        Vec3 operator*(float scalar) const;

        Vec3 operator/(float scalar) const;

        Vec3i &operator+=(const Vec3i &rhs);

        Vec3i &operator-=(const Vec3i &rhs);

        Vec3i &operator*=(int scalar);

        bool operator==(const Vec3i &rhs) const;

        bool operator!=(const Vec3i &rhs) const;

        int &operator[](int i);

        const int &operator[](int i) const;

        // Functions
        float length() const;

        int64_t lengthSq() const;

        Vec3 normalized() const;
    };

#pragma region functions

    int64_t dot(const Vec3i &v1, const Vec3i &v2);

    Vec3i cross(const Vec3i &v1, const Vec3i &v2);

    float distance(const Vec3i &v1, const Vec3i &v2);

    int64_t distanceSq(const Vec3i &v1, const Vec3i &v2);

    Vec3 lerp(const Vec3i &a, const Vec3i &b, float t);

#pragma endregion
}

template<>
struct std::hash<atlas::core::vec::Vec3i> {
    size_t operator()(const atlas::core::vec::Vec3i &v) const noexcept {
        size_t h1 = std::hash<int>{}(v.x);
        size_t h2 = std::hash<int>{}(v.y);
        size_t h3 = std::hash<int>{}(v.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
