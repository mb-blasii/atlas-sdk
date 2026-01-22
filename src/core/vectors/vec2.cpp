#include <atlas/core/vectors/vec2.h>

#include <cmath>
#include <cassert>
#include <cstdio>

namespace atlas::core::vec {

    // Constructors
    Vec2::Vec2() : x(0.0f), y(0.0f) {}

    Vec2::Vec2(float _x, float _y) : x(_x), y(_y) {}

    Vec2::Vec2(const Vec2& other) = default;

    // Operators
    Vec2 Vec2::operator+(const Vec2& rhs) const {
        return Vec2(x + rhs.x, y + rhs.y);
    }

    Vec2 Vec2::operator-(const Vec2& rhs) const {
        return Vec2(x - rhs.x, y - rhs.y);
    }

    Vec2 Vec2::operator*(float s) const {
        return Vec2(x * s, y * s);
    }

    Vec2 Vec2::operator/(float s) const {
        return Vec2(x / s, y / s);
    }

    Vec2& Vec2::operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vec2& Vec2::operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vec2& Vec2::operator*=(float s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vec2& Vec2::operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    float& Vec2::operator[](int i) {
        assert(i >= 0 && i < 2);
        return i == 0 ? x : y;
    }

    const float& Vec2::operator[](int i) const {
        assert(i >= 0 && i < 2);
        return i == 0 ? x : y;
    }

    // Functions
    float Vec2::length() const {
        return std::sqrt(x * x + y * y);
    }

    float Vec2::lengthSq() const {
        return x * x + y * y;
    }

    Vec2 Vec2::normalized() const {
        float l = length();
        return l != 0.0f ? Vec2(x / l, y / l) : Vec2(0.0f, 0.0f);
    }

    void Vec2::normalize() {
        float l = length();
        if (l != 0.0f) {
            x /= l;
            y /= l;
        }
    }

#pragma region functions

    float dot(const Vec2& v1, const Vec2& v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    float distance(const Vec2& v1, const Vec2& v2) {
        return (v1 - v2).length();
    }

    float distanceSq(const Vec2& v1, const Vec2& v2) {
        return (v1 - v2).lengthSq();
    }

    Vec2 lerp(const Vec2& a, const Vec2& b, float t) {
        return a + (b - a) * t;
    }

    char* toString(const Vec2& v) {
        static char buffer[40];
        std::snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", v.x, v.y);
        return buffer;
    }

#pragma endregion

}
