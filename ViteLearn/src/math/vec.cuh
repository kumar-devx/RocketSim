#pragma once

#include "base.cuh"

struct Vec3 {
    float x, y, z;

    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    RS_INLINE static Vec3 zero() { return Vec3(0, 0, 0); }
    RS_INLINE static Vec3 splat(float v) { return Vec3(v, v, v); }

    #ifdef __CUDA_ARCH__
    RS_INLINE Vec3 operator+(const Vec3& o) const { return Vec3(__fadd_rn(x, o.x), __fadd_rn(y, o.y), __fadd_rn(z, o.z)); }
    RS_INLINE Vec3 operator-(const Vec3& o) const { return Vec3(__fsub_rn(x, o.x), __fsub_rn(y, o.y), __fsub_rn(z, o.z)); }
    RS_INLINE Vec3 operator*(const Vec3& o) const { return Vec3(__fmul_rn(x, o.x), __fmul_rn(y, o.y), __fmul_rn(z, o.z)); }
    RS_INLINE Vec3 operator*(float s) const { return Vec3(__fmul_rn(x, s), __fmul_rn(y, s), __fmul_rn(z, s)); }
    RS_INLINE Vec3 operator/(float s) const { return Vec3(__fdiv_rn(x, s), __fdiv_rn(y, s), __fdiv_rn(z, s)); }
    #else
    RS_INLINE Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    RS_INLINE Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    RS_INLINE Vec3 operator*(const Vec3& o) const { return Vec3(x * o.x, y * o.y, z * o.z); }
    RS_INLINE Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    RS_INLINE Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
    #endif
    RS_INLINE Vec3 operator-() const { return Vec3(-x, -y, -z); }

    RS_INLINE float dot(const Vec3& o) const {
        return x * o.x + y * o.y + z * o.z;
    }

    RS_INLINE Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    RS_INLINE float length_squared() const { return dot(*this); }
    RS_INLINE float length() const { return sqrtf(length_squared()); }

    RS_INLINE float length_recip() const {
        #ifdef __CUDA_ARCH__
        return rsqrtf(length_squared());
        #else
        return 1.0f / sqrtf(length_squared());
        #endif
    }

    RS_INLINE Vec3 normalized() const {
        float len_sq = length_squared();
        if (len_sq <= 0) return zero();
        #ifdef __CUDA_ARCH__
        return *this * rsqrtf(len_sq);
        #else
        return *this * (1.0f / sqrtf(len_sq));
        #endif
    }

    RS_INLINE Vec3 normalize_or_zero() const {
        float rcp = length_recip();
        if (isfinite(rcp) && rcp > 0.0f) {
            return *this * rcp;
        }
        return zero();
    }

    RS_INLINE Vec3 abs() const { return Vec3(fabsf(x), fabsf(y), fabsf(z)); }

    RS_INLINE float& operator[](int i) { return (&x)[i]; }
    RS_INLINE float operator[](int i) const { return (&x)[i]; }
};

struct Vec4 {
    float x, y, z, w;

    RS_INLINE Vec4() : x(0), y(0), z(0), w(0) {}
    RS_INLINE Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    RS_INLINE int max_position() const {
        int idx = 0; float m = x;
        if (y > m) { m = y; idx = 1; }
        if (z > m) { m = z; idx = 2; }
        if (w > m) { idx = 3; }
        return idx;
    }
};
