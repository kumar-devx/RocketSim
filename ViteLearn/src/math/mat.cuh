#pragma once

#include "quat.cuh"

struct Mat3 {
    Vec3 x_axis, y_axis, z_axis;

    RS_INLINE Mat3() : x_axis(Vec3(1,0,0)), y_axis(Vec3(0,1,0)), z_axis(Vec3(0,0,1)) {}
    RS_INLINE Mat3(const Vec3& x, const Vec3& y, const Vec3& z) : x_axis(x), y_axis(y), z_axis(z) {}
    RS_INLINE static Mat3 identity() { return Mat3(); }

    RS_INLINE Vec3 col(int i) const { return (&x_axis)[i]; }

    RS_INLINE Vec3 operator*(const Vec3& v) const {
        Vec3 res = x_axis * v.x;
        res = res + y_axis * v.y;
        res = res + z_axis * v.z;
        return res;
    }

    RS_INLINE Mat3 operator*(const Mat3& o) const {
        return Mat3(*this * o.x_axis, *this * o.y_axis, *this * o.z_axis);
    }

    RS_INLINE Mat3 transpose() const {
        return Mat3(
            Vec3(x_axis.x, y_axis.x, z_axis.x),
            Vec3(x_axis.y, y_axis.y, z_axis.y),
            Vec3(x_axis.z, y_axis.z, z_axis.z)
        );
    }

    RS_INLINE Mat3 abs() const { return Mat3(x_axis.abs(), y_axis.abs(), z_axis.abs()); }

    RS_INLINE float cofac(int r1, int c1, int r2, int c2) const {
        return col(r1)[c1] * col(r2)[c2] - col(r1)[c2] * col(r2)[c1];
    }

    RS_INLINE Mat3 bullet_inverse() const {
        Vec3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
        float det = x_axis.dot(co);
        if (fabsf(det) < 1e-12f) return identity();
        float s = 1.0f / det;

        return Mat3(
            co * s,
            Vec3(cofac(0, 2, 2, 1), cofac(0, 0, 2, 2), cofac(0, 1, 2, 0)) * s,
            Vec3(cofac(0, 1, 1, 2), cofac(0, 2, 1, 0), cofac(0, 0, 1, 1)) * s
        );
    }
};

RS_INLINE Mat3 mat3_from_quat(const Quat& q) {
    float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!(d > 1e-12f)) return Mat3::identity();
    float s = 2.0f / d;

    float xs = q.x * s, ys = q.y * s, zs = q.z * s;
    float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
    float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
    float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;

    return Mat3(
        Vec3(1.0f - (yy + zz), xy + wz, xz - wy),
        Vec3(xy - wz, 1.0f - (xx + zz), yz + wx),
        Vec3(xz + wy, yz - wx, 1.0f - (xx + yy))
    );
}

RS_INLINE Mat3 bullet_mat3_from_quat(const Quat& q) {
    return mat3_from_quat(q);
}

RS_INLINE Quat bullet_mul_quat(const Quat& q1, const Quat& q2) {
    float a2x = q1.y * q2.z;
    float a2y = q1.z * q2.x;
    float a2z = q1.x * q2.y;
    float a2w = q1.y * q2.y;

    float a1x = q1.x * q2.w + a2x;
    float a1y = q1.y * q2.w + a2y;
    float a1z = q1.z * q2.w + a2z;
    float a1w = q1.x * q2.x + a2w;

    float b1x = q1.z * q2.y;
    float b1y = q1.x * q2.z;
    float b1z = q1.y * q2.x;
    float b1w = q1.z * q2.z;

    float a0x = q1.w * q2.x - b1x;
    float a0y = q1.w * q2.y - b1y;
    float a0z = q1.w * q2.z - b1z;
    float a0w = q1.w * q2.w - b1w;

    return Quat(
        a0x + a1x,
        a0y + a1y,
        a0z + a1z,
        a0w - a1w
    );
}

RS_INLINE Quat bullet_normalize(const Quat& q) {
    float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (len_sq > SIMD_EPSILON) {
        float rcp = rsqrtf(len_sq);
        return Quat(q.x * rcp, q.y * rcp, q.z * rcp, q.w * rcp);
    }
    return Quat(0, 0, 0, 1);
}

struct Affine3 {
    Mat3 matrix3;
    Vec3 translation;

    RS_INLINE Affine3() : matrix3(Mat3::identity()), translation(Vec3::zero()) {}
    RS_INLINE Affine3(const Mat3& m, const Vec3& t) : matrix3(m), translation(t) {}
    RS_INLINE static Affine3 identity() { return Affine3(); }

    RS_INLINE Vec3 transform_point(const Vec3& p) const { return matrix3 * p + translation; }

    RS_INLINE Vec3 inv_xform(const Vec3& p) const {
        return matrix3.transpose() * (p - translation);
    }

    RS_INLINE Affine3 transpose() const {
        Mat3 m = matrix3.transpose();
        return Affine3(m, m * (translation * -1.0f));
    }

    RS_INLINE Affine3 operator*(const Affine3& o) const {
        return Affine3(matrix3 * o.matrix3, transform_point(o.translation));
    }
};
