#pragma once

#include <cstdint>

#include "vec2.h"

namespace atlas::core::vec {

    struct Vec2i {
        int x;
        int y;

        // Constructors
        Vec2i();
        Vec2i(int _x, int _y);
        Vec2i(const Vec2i& other);

        // Operators
        Vec2i operator+(const Vec2i& rhs) const;
        Vec2i operator-(const Vec2i& rhs) const;
        Vec2i operator*(int scalar) const;
        Vec2i operator/(int scalar) const;
        Vec2 operator*(float scalar) const;
        Vec2 operator/(float scalar) const;

        Vec2i& operator+=(const Vec2i& rhs);
        Vec2i& operator-=(const Vec2i& rhs);
        Vec2i& operator*=(int scalar);

        bool operator==(const Vec2i& rhs) const;
        bool operator!=(const Vec2i& rhs) const;

        int& operator[](int i);
        const int& operator[](int i) const;

        // Functions
        float length() const;
        int64_t lengthSq() const;
    };

#pragma region functions

    int64_t dot(const Vec2i& v1, const Vec2i& v2);

    float distance(const Vec2i& v1, const Vec2i& v2);
    int64_t distanceSq(const Vec2i& v1, const Vec2i& v2);

    Vec2 lerp(const Vec2i& a, const Vec2i& b, float t);

#pragma endregion

}
