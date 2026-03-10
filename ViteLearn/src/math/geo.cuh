#pragma once

#include "mat.cuh"

constexpr float ANGULAR_MOTION_THRESHOLD = 0.7853981633974483f;

struct Aabb {
    Vec3 min, max;

    RS_INLINE Aabb() : min(Vec3::splat(LARGE_FLOAT)), max(Vec3::splat(-LARGE_FLOAT)) {}
    RS_INLINE Aabb(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}

    RS_INLINE bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }

    RS_INLINE bool intersects(const Aabb& o) const { return overlaps(o); }

    RS_INLINE Vec3 center() const { return (min + max) * 0.5f; }
    RS_INLINE Vec3 extents() const { return (max - min) * 0.5f; }

    RS_INLINE Aabb merge(const Aabb& o) const {
        return Aabb(
            Vec3(fminf(min.x, o.min.x), fminf(min.y, o.min.y), fminf(min.z, o.min.z)),
            Vec3(fmaxf(max.x, o.max.x), fmaxf(max.y, o.max.y), fmaxf(max.z, o.max.z))
        );
    }

    RS_INLINE float area() const {
        Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    RS_INLINE bool ray_intersect(const Vec3& origin, const Vec3& inv_dir, float& tmin, float& tmax) const {
        float t1 = (min.x - origin.x) * inv_dir.x;
        float t2 = (max.x - origin.x) * inv_dir.x;
        tmin = fminf(t1, t2);
        tmax = fmaxf(t1, t2);

        t1 = (min.y - origin.y) * inv_dir.y;
        t2 = (max.y - origin.y) * inv_dir.y;
        tmin = fmaxf(tmin, fminf(t1, t2));
        tmax = fminf(tmax, fmaxf(t1, t2));

        t1 = (min.z - origin.z) * inv_dir.z;
        t2 = (max.z - origin.z) * inv_dir.z;
        tmin = fmaxf(tmin, fminf(t1, t2));
        tmax = fminf(tmax, fmaxf(t1, t2));

        return tmax >= tmin && tmax >= 0;
    }

    RS_INLINE static Aabb from_triangle(const Vec3& p0, const Vec3& p1, const Vec3& p2) {
        return Aabb(
            Vec3(fminf(fminf(p0.x, p1.x), p2.x), fminf(fminf(p0.y, p1.y), p2.y), fminf(fminf(p0.z, p1.z), p2.z)),
            Vec3(fmaxf(fmaxf(p0.x, p1.x), p2.x), fmaxf(fmaxf(p0.y, p1.y), p2.y), fmaxf(fmaxf(p0.z, p1.z), p2.z))
        );
    }
};

RS_INLINE void plane_space(const Vec3& n, Vec3& p, Vec3& q) {
    if (fabsf(n.z) > 0.7071067811865476f) {
        float a = n.y * n.y + n.z * n.z;
        float k = rsqrtf(a);
        p = Vec3(0.0f, -n.z * k, n.y * k);
        q = Vec3(a * k, -n.x * p.z, n.x * p.y);
    } else {
        float a = n.x * n.x + n.y * n.y;
        float k = rsqrtf(a);
        p = Vec3(-n.y * k, n.x * k, 0.0f);
        q = Vec3(-n.z * p.y, n.z * p.x, a * k);
    }
}

RS_INLINE Vec3 plane_space_1(const Vec3& n) {
    if (fabsf(n.z) > 0.7071067811865476f) {
        float a = n.y * n.y + n.z * n.z;
        float k = rsqrtf(a);
        return Vec3(0.0f, -n.z * k, n.y * k);
    } else {
        float a = n.x * n.x + n.y * n.y;
        float k = rsqrtf(a);
        return Vec3(-n.y * k, n.x * k, 0.0f);
    }
}

RS_INLINE Vec3 interpolate_3(const Vec3& v0, const Vec3& v1, float rt) {
    float s = 1.0f - rt;
    return v0 * s + v1 * rt;
}

RS_INLINE Quat quat_from_mat3(const Mat3& m) {
    float trace = m.x_axis.x + m.y_axis.y + m.z_axis.z;
    if (!isfinite(trace)) return Quat(0, 0, 0, 1);

    if (trace > 0.0f) {
        float t = trace + 1.0f;
        float s = 0.5f * rsqrtf(t);
        return Quat((m.y_axis.z - m.z_axis.y) * s, (m.z_axis.x - m.x_axis.z) * s, (m.x_axis.y - m.y_axis.x) * s, t * s);
    }

    if (m.x_axis.x >= m.y_axis.y && m.x_axis.x >= m.z_axis.z) {
        float t = fmaxf(m.x_axis.x - m.y_axis.y - m.z_axis.z + 1.0f, 0.0f);
        float s = (t > 1e-10f) ? 0.5f * rsqrtf(t) : 0.0f;
        return Quat(t * s, (m.x_axis.y + m.y_axis.x) * s, (m.x_axis.z + m.z_axis.x) * s, (m.y_axis.z - m.z_axis.y) * s);
    }

    if (m.y_axis.y >= m.z_axis.z) {
        float t = fmaxf(m.y_axis.y - m.x_axis.x - m.z_axis.z + 1.0f, 0.0f);
        float s = (t > 1e-10f) ? 0.5f * rsqrtf(t) : 0.0f;
        return Quat((m.x_axis.y + m.y_axis.x) * s, t * s, (m.y_axis.z + m.z_axis.y) * s, (m.z_axis.x - m.x_axis.z) * s);
    }

    float t = fmaxf(m.z_axis.z - m.x_axis.x - m.y_axis.y + 1.0f, 0.0f);
    float s = (t > 1e-10f) ? 0.5f * rsqrtf(t) : 0.0f;
    return Quat((m.x_axis.z + m.z_axis.x) * s, (m.y_axis.z + m.z_axis.y) * s, t * s, (m.x_axis.y - m.y_axis.x) * s);
}

RS_INLINE void integrate_transform_no_rot(Affine3& t, const Vec3& lin_vel, float dt) {
    Vec3 new_trans = t.translation + lin_vel * dt;
    if (isfinite(new_trans.x) && isfinite(new_trans.y) && isfinite(new_trans.z)) {
        t.translation = new_trans;
    }
}

RS_INLINE void integrate_transform(Affine3& t, const Vec3& lin_vel, const Vec3& ang_vel, float dt) {
    integrate_transform_no_rot(t, lin_vel, dt);

    float angle2 = ang_vel.length_squared();
    if (angle2 < SIMD_EPSILON) return;
    float angle = sqrtf(angle2);

    if (angle * dt > ANGULAR_MOTION_THRESHOLD) {
        angle = ANGULAR_MOTION_THRESHOLD / dt;
    }

    Vec3 axis;
    if (angle < 0.001f) {
        axis = ang_vel * (0.5f * dt - dt * dt * dt * 0.020833334f) * angle * angle;
    } else {
        axis = ang_vel * (sinf(0.5f * angle * dt) / angle);
    }

    Quat dorn(axis.x, axis.y, axis.z, cosf(angle * dt * 0.5f));
    Quat orn0 = quat_from_mat3(t.matrix3);
    Quat predicted_orn = bullet_normalize(bullet_mul_quat(dorn, orn0));
    t.matrix3 = mat3_from_quat(predicted_orn);
}

RS_INLINE Aabb transform_aabb(const Aabb& local_aabb, const Affine3& trans) {
    Vec3 local_half = (local_aabb.max - local_aabb.min) * 0.5f;
    Vec3 local_center = (local_aabb.max + local_aabb.min) * 0.5f;

    Mat3 abs_b = trans.matrix3.abs();
    Vec3 center = trans.transform_point(local_center);
    Vec3 extent = abs_b * local_half;

    return Aabb(center - extent, center + extent);
}
