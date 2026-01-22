#include <atlas/core/matrix/mat4.h>

namespace atlas::core::mat4 {

    // Constructors
    Mat4::Mat4() { *this = identity(); }

    Mat4::Mat4(const float diag) {
        for (float &i: m) i = 0.0f;
        m[0] = m[5] = m[10] = m[15] = diag;
    }

    Mat4::Mat4(const Mat4 &other) { *this = other; }

    // Operators
    Mat4& Mat4::operator=(const Mat4 &other) {
        if (this != &other) {
            for (int i = 0; i < 16; i++) m[i] = other.m[i];
        }
        return *this;
    }

    float& Mat4::operator()(int row, int col) { return m[row * 4 + col]; }
    const float& Mat4::operator()(int row, int col) const { return m[row * 4 + col]; }


#pragma region functions

    Mat4 identity() {
        Mat4 mat;

        for (float &i: mat.m) i = 0.0f;
        mat.m[0] = mat.m[5] = mat.m[10] = mat.m[15] = 1.0f;

        return mat;
    }

    Mat4 mul(const Mat4 &a, const Mat4 &b) {
        Mat4 result;
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                    sum += a(row, k) * b(k, col);
                result(row, col) = sum;
            }
        return result;
    }

    Mat4 translate(const vec::Vec3 &t) {
        Mat4 mat = identity();
        mat(0, 3) = t.x;
        mat(1, 3) = t.y;
        mat(2, 3) = t.z;
        return mat;
    }

    Mat4 scale(const vec::Vec3 &s) {
        Mat4 mat = identity();
        mat(0, 0) = s.x;
        mat(1, 1) = s.y;
        mat(2, 2) = s.z;
        return mat;
    }

    Mat4 rotate(const quat::Quat &r) {
        Mat4 mat = identity();

        float xx = r.x * r.x;
        float yy = r.y * r.y;
        float zz = r.z * r.z;
        float xy = r.x * r.y;
        float xz = r.x * r.z;
        float yz = r.y * r.z;
        float wx = r.w * r.x;
        float wy = r.w * r.y;
        float wz = r.w * r.z;

        mat(0,0) = 1 - 2*(yy + zz);
        mat(0,1) = 2*(xy - wz);
        mat(0,2) = 2*(xz + wy);

        mat(1,0) = 2*(xy + wz);
        mat(1,1) = 1 - 2*(xx + zz);
        mat(1,2) = 2*(yz - wx);

        mat(2,0) = 2*(xz - wy);
        mat(2,1) = 2*(yz + wx);
        mat(2,2) = 1 - 2*(xx + yy);

        return mat;
    }

    Mat4 rotate(const vec::Vec3 &r) {
        quat::Quat q = quat::fromEuler(r);
        return rotate(q);
    }

    Mat4 rotate(float x, float y, float z) {
        vec::Vec3 t{x, y, z};
        return rotate(t);
    }

    Mat4 TRS(const vec::Vec3 &t, const quat::Quat &r, const vec::Vec3 &s) {
        Mat4 S = scale(s);
        Mat4 R = rotate(r);
        Mat4 T = translate(t);

        return mul(T, mul(R, S));
    }

    Mat4 TRS(const vec::Vec3 &t, const vec::Vec3 &r, const vec::Vec3 &s) {
        quat::Quat q = quat::fromEuler(r);
        return TRS(t, q, s);
    }

    vec::Vec3 transformPoint(const Mat4 &mat, const vec::Vec3 &v) {
        float x = mat(0, 0) * v.x + mat(0, 1) * v.y + mat(0, 2) * v.z + mat(0, 3);
        float y = mat(1, 0) * v.x + mat(1, 1) * v.y + mat(1, 2) * v.z + mat(1, 3);
        float z = mat(2, 0) * v.x + mat(2, 1) * v.y + mat(2, 2) * v.z + mat(2, 3);
        float w = mat(3, 0) * v.x + mat(3, 1) * v.y + mat(3, 2) * v.z + mat(3, 3);
        if (w != 0.0f && w != 1.0f) {
            x /= w;
            y /= w;
            z /= w;
        }
        return vec::Vec3{x, y, z};
    }

    vec::Vec3 transformDirection(const Mat4 &mat, const vec::Vec3 &v) {
        float x = mat(0, 0) * v.x + mat(0, 1) * v.y + mat(0, 2) * v.z;
        float y = mat(1, 0) * v.x + mat(1, 1) * v.y + mat(1, 2) * v.z;
        float z = mat(2, 0) * v.x + mat(2, 1) * v.y + mat(2, 2) * v.z;
        return vec::Vec3{x, y, z};
    }

    Mat4 transpose(const Mat4 &mat) {
        Mat4 result;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                result(r, c) = mat(c, r);
        return result;
    }

#pragma endregion

}