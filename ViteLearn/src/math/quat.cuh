#pragma once

#include "vec.cuh"

struct Quat {
    float x, y, z, w;

    RS_INLINE Quat() : x(0), y(0), z(0), w(1) {}
    RS_INLINE Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    RS_INLINE static Quat identity() { return Quat(0, 0, 0, 1); }

    RS_INLINE Quat operator*(const Quat& o) const {
        return Quat(
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y + y * o.w + z * o.x - x * o.z,
            w * o.z + z * o.w + x * o.y - y * o.x,
            w * o.w - x * o.x - y * o.y - z * o.z
        );
    }
};

RS_INLINE Quat quat_from_axis_angle(const Vec3& axis, float angle) {
    float half = angle * 0.5f;
    float s = sinf(half);
    return Quat(axis.x * s, axis.y * s, axis.z * s, cosf(half));
}
