#pragma once
#include <cmath>
#include <cstdio>
#include <cassert>

namespace atlas::core::vec {

    struct Vec2 {
        float x, y;

        // Constructors
        Vec2() : x(0), y(0) {}
        Vec2(float _x, float _y) : x(_x), y(_y) {}
        Vec2(const Vec2& other) = default;

        // Operators
        Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
        Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
        Vec2 operator*(float s) const { return {x * s, y * s}; }
        Vec2 operator/(float s) const { return {x / s, y / s}; }

        Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
        Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
        Vec2& operator*=(float s) { x *= s; y *= s; return *this; }
        Vec2& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }

        float& operator[](int i) {
            assert(i>=0 && i<2);
            if(i==0) return x;
            return y;
        }

        const float& operator[](int i) const {
            assert(i>=0 && i<2);
            if(i==0) return x;
            return y;
        }

        // Functions
        float length() const { return std::sqrt(x * x + y * y); }
        float lengthSq() const { return x * x + y * y; }
        Vec2 normalized() const { float l = length(); return l != 0 ? Vec2(x / l, y / l) : Vec2(0, 0); }
        void normalize() { float l = length(); if (l != 0) { x /= l; y /= l; } }

    };

#pragma region functions

    inline float dot(const Vec2& v1, const Vec2& v2) { return v1.x * v2.x + v1.y * v2.y; }

    inline float distance(const Vec2& v1, const Vec2& v2) { return (v1 - v2).length(); }
    inline float distanceSq(const Vec2& v1, const Vec2& v2) { return (v1 - v2).lengthSq(); }

    inline Vec2 lerp(const Vec2& a, const Vec2& b, float t) { return a + (b - a) * t; }

    inline char* toString(const Vec2& v) {
        static char buffer[40];
        snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", v.x, v.y);
        return buffer;
    }

#pragma endregion

}
