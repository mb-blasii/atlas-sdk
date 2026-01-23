#include <atlas/core/vectors/vec2i.h>

#include <cmath>
#include <cassert>

namespace atlas::core::vec {

    // Constructors
    Vec2i::Vec2i() : x(0), y(0) {}
    Vec2i::Vec2i(int _x, int _y) : x(_x), y(_y) {}
    Vec2i::Vec2i(const Vec2i& other) = default;

    // Operators
    Vec2i Vec2i::operator+(const Vec2i& rhs) const { return { x + rhs.x, y + rhs.y }; }
    Vec2i Vec2i::operator-(const Vec2i& rhs) const { return { x - rhs.x, y - rhs.y }; }
    Vec2i Vec2i::operator*(int scalar) const { return { x * scalar, y * scalar }; }
    Vec2i Vec2i::operator/(int scalar) const { return { x / scalar, y / scalar }; }
    Vec2 Vec2i::operator*(float scalar) const { return { x * scalar, y * scalar }; }
    Vec2 Vec2i::operator/(float scalar) const { return { x / scalar, y / scalar }; }

    Vec2i& Vec2i::operator+=(const Vec2i& rhs) { x += rhs.x; y += rhs.y; return *this; }
    Vec2i& Vec2i::operator-=(const Vec2i& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    Vec2i& Vec2i::operator*=(int scalar) { x *= scalar; y *= scalar; return *this; }

    bool Vec2i::operator==(const Vec2i& rhs) const { return x == rhs.x && y == rhs.y; }
    bool Vec2i::operator!=(const Vec2i& rhs) const { return !(*this == rhs); }

    int& Vec2i::operator[](int i) {
        assert(i >= 0 && i < 2);
        return (i == 0) ? x : y;
    }

    const int& Vec2i::operator[](int i) const {
        assert(i >= 0 && i < 2);
        return (i == 0) ? x : y;
    }

    // Functions
    float Vec2i::length() const {
        return std::sqrt(static_cast<float>(x * x + y * y));
    }

    int64_t Vec2i::lengthSq() const {
        return x * x + y * y;
    }

#pragma region functions

    int64_t dot(const Vec2i& v1, const Vec2i& v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    float distance(const Vec2i& v1, const Vec2i& v2) {
        return (v1 - v2).length();
    }

    int64_t distanceSq(const Vec2i& v1, const Vec2i& v2) {
        return (v1 - v2).lengthSq();
    }

    Vec2 lerp(const Vec2i &a, const Vec2i &b, float t) {
        return Vec2(
            static_cast<float>(a.x) + (static_cast<float>(b.x - a.x) * t),
            static_cast<float>(a.y) + (static_cast<float>(b.y - a.y) * t)
        );
    }

#pragma endregion

}
