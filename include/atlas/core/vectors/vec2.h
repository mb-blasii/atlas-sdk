#pragma once

namespace atlas::core::vec {

    struct Vec2 {
        float x, y;

        // Constructors
        Vec2();
        Vec2(float _x, float _y);
        Vec2(const Vec2& other);

        // Operators
        Vec2 operator+(const Vec2& rhs) const;
        Vec2 operator-(const Vec2& rhs) const;
        Vec2 operator*(float s) const;
        Vec2 operator/(float s) const;

        Vec2& operator+=(const Vec2& rhs);
        Vec2& operator-=(const Vec2& rhs);
        Vec2& operator*=(float s);
        Vec2& operator/=(float scalar);

        float& operator[](int i);
        const float& operator[](int i) const;

        // Functions
        float length() const;
        float lengthSq() const;
        Vec2 normalized() const;
        void normalize();

    };

#pragma region functions

    float dot(const Vec2& v1, const Vec2& v2);

    float distance(const Vec2& v1, const Vec2& v2);
    float distanceSq(const Vec2& v1, const Vec2& v2);

    Vec2 lerp(const Vec2& a, const Vec2& b, float t);

    char* toString(const Vec2& v);

#pragma endregion

}
