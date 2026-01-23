#include <atlas/core/vectors/vec3.h>

#include <cmath>
#include <cassert>

namespace atlas::core::vec {

    // Constructors
    Vec3::Vec3() : x(0), y(0), z(0) {}
    Vec3::Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    Vec3::Vec3(const Vec3& other) = default;

    // Operators
    Vec3 Vec3::operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 Vec3::operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 Vec3::operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3 Vec3::operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }

    Vec3& Vec3::operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& Vec3::operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& Vec3::operator*=(float scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vec3& Vec3::operator/=(float scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    float& Vec3::operator[](int i) {
        assert(i>=0 && i<3);
        if(i==0) return x;
        if(i==1) return y;
        return z;
    }

    const float& Vec3::operator[](int i) const {
        assert(i>=0 && i<3);
        if(i==0) return x;
        if(i==1) return y;
        return z;
    }

    // Functions
    float Vec3::length() const { return std::sqrt(x * x + y * y + z * z); }
    float Vec3::lengthSq() const { return x * x + y * y + z * z; }
    Vec3 Vec3::normalized() const { float l = length(); return l != 0 ? Vec3(x / l, y / l, z / l) : Vec3(0, 0, 0); }
    void Vec3::normalize() { float l = length(); if (l != 0) { x /= l; y /= l; z /= l; } }

#pragma region functions

    float dot(const Vec3& v1, const Vec3& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
    Vec3 cross(const Vec3& v1, const Vec3& v2) { return Vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x); }

    float distance(const Vec3& v1, const Vec3& v2) { return (v1 - v2).length(); }
    float distanceSq(const Vec3& v1, const Vec3& v2) { return (v1 - v2).lengthSq(); }

    Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }

#pragma endregion

}