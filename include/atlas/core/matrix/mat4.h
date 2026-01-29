#pragma once
#include <atlas/core/vectors/vec3.h>
#include <atlas/core/quaternion/quaternion.h>

namespace atlas::core::mat4 {

    struct Mat4 {
        float m[16]{};

        // Constructors
        Mat4();
        explicit Mat4(float diag);
        Mat4(const Mat4 &other);

        // Operators
        Mat4 &operator=(const Mat4 &other);

        float& operator()(int row, int col);
        const float& operator()(int row, int col) const;

    };

#pragma region functions

    Mat4 identity();

    Mat4 mul(const Mat4 &a, const Mat4 &b);

    Mat4 translate(const vec::Vec3 &t);

    Mat4 scale(const vec::Vec3 &s);

    Mat4 rotate(const quat::Quat &r);
    Mat4 rotate(const vec::Vec3 &r);
    Mat4 rotate(float x, float y, float z);

    // TRS specific functions

    Mat4 TRS(const vec::Vec3 &t, const quat::Quat &r, const vec::Vec3 &s);
    Mat4 TRS(const vec::Vec3 &t, const vec::Vec3 &r, const vec::Vec3 &s);

    Mat4 inverseTRS(const Mat4& m);
    Mat4 inverseTRS(const vec::Vec3& t, const quat::Quat& r, const vec::Vec3& s);

    vec::Vec3 getTranslation(const Mat4 &trs);
    quat::Quat getRotation(const Mat4 &trs);
    vec::Vec3 getScale(const Mat4 &trs);

    void decomposeTRS(const Mat4& m, vec::Vec3& outT, quat::Quat& outR, vec::Vec3& outS);

    vec::Vec3 transformPoint(const Mat4 &mat, const vec::Vec3 &v);
    vec::Vec3 transformDirection(const Mat4 &mat, const vec::Vec3 &v);

    Mat4 transpose(const Mat4 &mat);

#pragma endregion
}
