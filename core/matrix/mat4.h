#pragma once
#include <cmath>
#include <vectors/vec3.h>

namespace atlas::core::mat4 {
    struct Mat4 {
        float m[16]{};

        // Constructors
        Mat4() { setIdentity(); }

        explicit Mat4(const float diag) {
            for (float &i: m) i = 0.0f;
            m[0] = m[5] = m[10] = m[15] = diag;
        }

        Mat4(const Mat4 &other) { *this = other; }

        // Operators
        Mat4 &operator=(const Mat4 &other) {
            if (this != &other) {
                for (int i = 0; i < 16; i++) m[i] = other.m[i];
            }
            return *this;
        }

        float &operator()(int row, int col) { return m[row * 4 + col]; }
        const float &operator()(int row, int col) const { return m[row * 4 + col]; }

        // Identity
        void setIdentity() {
            for (float &i: m) i = 0.0f;
            m[0] = m[5] = m[10] = m[15] = 1.0f;
        }
    };

    // Functions
    inline Mat4 identity() {
        Mat4 mat;
        mat.setIdentity();
        return mat;
    }

    inline Mat4 mul(const Mat4 &a, const Mat4 &b) {
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

    inline Mat4 translate(const vec::Vec3 &t) {
        Mat4 mat = identity();
        mat(0, 3) = t.x;
        mat(1, 3) = t.y;
        mat(2, 3) = t.z;
        return mat;
    }

    inline Mat4 scale(const vec::Vec3 &s) {
        Mat4 mat = identity();
        mat(0, 0) = s.x;
        mat(1, 1) = s.y;
        mat(2, 2) = s.z;
        return mat;
    }

    inline Mat4 rotate(const vec::Vec3 &r) {
        float cx = std::cos(r.x), sx = std::sin(r.x);
        float cy = std::cos(r.y), sy = std::sin(r.y);
        float cz = std::cos(r.z), sz = std::sin(r.z);

        Mat4 mat = identity();

        mat(0,0) = cy * cz;
        mat(0,1) = -cy * sz;
        mat(0,2) = sy;

        mat(1,0) = sx * sy * cz + cx * sz;
        mat(1,1) = -sx * sy * sz + cx * cz;
        mat(1,2) = -sx * cy;

        mat(2,0) = -cx * sy * cz + sx * sz;
        mat(2,1) = cx * sy * sz + sx * cz;
        mat(2,2) = cx * cy;

        return mat;
    }

    inline Mat4 rotate(float x, float y, float z) {
        vec::Vec3 t{x, y, z};
        return rotate(t);
    }

    inline Mat4 TRS(const vec::Vec3 &t, const vec::Vec3 &r, const vec::Vec3 &s) {
        Mat4 S = scale(s);
        Mat4 R = rotate(r);
        Mat4 T = translate(t);

        return mul(T, mul(R, S));
    }

    inline vec::Vec3 transformPoint(const Mat4 &mat, const vec::Vec3 &v) {
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

    inline vec::Vec3 transformDirection(const Mat4 &mat, const vec::Vec3 &v) {
        float x = mat(0, 0) * v.x + mat(0, 1) * v.y + mat(0, 2) * v.z;
        float y = mat(1, 0) * v.x + mat(1, 1) * v.y + mat(1, 2) * v.z;
        float z = mat(2, 0) * v.x + mat(2, 1) * v.y + mat(2, 2) * v.z;
        return vec::Vec3{x, y, z};
    }

    inline Mat4 transpose(const Mat4 &mat) {
        Mat4 result;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                result(r, c) = mat(c, r);
        return result;
    }
}
