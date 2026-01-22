#pragma once
#include <atlas/core/vectors/vec3.h>

namespace atlas::core::quat {

    struct Quat {
        float x, y, z, w;

        // Constructors
        Quat();
        Quat(float _x, float _y, float _z, float _w);
        Quat(const Quat &other);

        // Operators
#pragma region operators

        Quat operator+(const Quat &rhs) const;
        Quat operator-(const Quat &rhs) const;
        Quat operator*(const Quat &rhs) const;

        Quat operator*(float s) const;
        Quat operator/(float s) const;

        Quat& operator+=(const Quat &rhs);
        Quat& operator-=(const Quat &rhs);
        Quat& operator*=(const Quat &rhs);

        // Optional: compound assignment
        Quat& operator*=(float s);
        Quat& operator/=(float s);

        Quat operator-() const;

        bool operator==(const Quat &rhs) const;
        bool operator!=(const Quat &rhs) const;

#pragma endregion

        // Normalize
        float length() const;
        float lengthSq() const;

        void normalize();
        Quat normalized() const;
    };

#pragma region quaternion functions

    Quat identity();

    // Conjugate & Inverse
    Quat conjugate(const Quat &q);

    Quat inverse(const Quat &q);

    // From/To Euler angles (XYZ order, radians)
    Quat fromEuler(const vec::Vec3 &euler);
    vec::Vec3 toEuler(const Quat &q);

    // Lerp (linear interpolation)
    Quat lerp(const Quat &a, const Quat &b, float t);

    // Slerp (spherical linear interpolation)
    Quat slerp(const Quat &qa, const Quat &qb, float t);

    // Rotate a vector
    vec::Vec3 rotate(const vec::Vec3 &v, const Quat &q);

#pragma endregion
}
