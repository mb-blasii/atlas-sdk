#pragma once

namespace atlas::core::vec {

    struct Vec3 {
        float x, y, z;

        // Constructors
        Vec3();
        Vec3(float _x, float _y, float _z);
        Vec3(const Vec3& other);

        // Operators
        Vec3 operator+(const Vec3& rhs) const;
        Vec3 operator-(const Vec3& rhs) const;
        Vec3 operator*(float scalar) const;
        Vec3 operator/(float scalar) const;

        Vec3& operator+=(const Vec3& rhs);
        Vec3& operator-=(const Vec3& rhs);
        Vec3& operator*=(float scalar);
        Vec3& operator/=(float scalar);

        float& operator[](int i);
        const float& operator[](int i) const;

        // Functions
        float length() const;
        float lengthSq() const;
        Vec3 normalized() const;
        void normalize();

    };

#pragma region functions

    float dot(const Vec3& v1, const Vec3& v2);
    Vec3 cross(const Vec3& v1, const Vec3& v2);

    float distance(const Vec3& v1, const Vec3& v2);
    float distanceSq(const Vec3& v1, const Vec3& v2);

    Vec3 lerp(const Vec3& a, const Vec3& b, float t);

#pragma endregion

}
