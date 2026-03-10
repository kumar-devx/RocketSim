#pragma once

#include "shapes.cuh"

constexpr float BASE_CONTACT_BREAKING_THRESHOLD = 0.02f;
constexpr int MANIFOLD_CACHE_SIZE = 4;

enum ManifoldType {
    MANIFOLD_SPHERE_SPHERE = 0,
    MANIFOLD_SPHERE_BOX,
    MANIFOLD_SPHERE_MESH,
    MANIFOLD_BOX_BOX,
    MANIFOLD_BOX_MESH,
    MANIFOLD_PLANE
};

struct ManifoldPoint {
    Vec3 local_a, local_b;
    Vec3 world_a, world_b;
    Vec3 normal;
    float distance;
    float friction, restitution;
    bool is_special;

    RS_INLINE ManifoldPoint()
        : distance(0), friction(0), restitution(0), is_special(false) {}

    RS_INLINE ManifoldPoint(const Vec3& la, const Vec3& lb, const Vec3& n, float d)
        : local_a(la), local_b(lb), normal(n), distance(d), friction(0), restitution(0),
          is_special(false) {}
};

struct PersistentManifold {
    ManifoldPoint points[MANIFOLD_CACHE_SIZE];
    int num_points;
    int body0_idx, body1_idx;
    float breaking_threshold;
    float processing_threshold;
    ManifoldType type;
    bool is_swapped;
    bool is_active;

    RS_INLINE PersistentManifold() : num_points(0), body0_idx(-1), body1_idx(-1),
        breaking_threshold(BASE_CONTACT_BREAKING_THRESHOLD), processing_threshold(1e18f),
        type(MANIFOLD_SPHERE_SPHERE), is_swapped(false), is_active(false) {}

    RS_INLINE bool should_persist() const {
        return type == MANIFOLD_SPHERE_BOX || type == MANIFOLD_BOX_BOX ||
               type == MANIFOLD_SPHERE_MESH || type == MANIFOLD_BOX_MESH ||
               type == MANIFOLD_PLANE;
    }

    RS_INLINE static float calc_friction(bool a_static, bool b_static, float fa, float fb) {
        return (a_static || b_static) ? fminf(fa, fb) : fa * fb;
    }

    RS_INLINE static float calc_restitution(bool a_static, bool b_static, float ra, float rb) {
        return (a_static || b_static) ? fmaxf(ra, rb) : ra * rb;
    }

    RS_INLINE static float get_res(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
        return (p - a).cross(b - c).length_squared();
    }

    RS_INLINE bool valid_contact_distance(const ManifoldPoint& pt) const {
        return pt.distance <= breaking_threshold;
    }

    RS_INLINE int sort_cached(const ManifoldPoint& cp) const {
        int max_idx = MANIFOLD_CACHE_SIZE;
        float max_pen = cp.distance;
        for (int i = 0; i < num_points; i++) {
            if (points[i].distance < max_pen) { max_idx = i; max_pen = points[i].distance; }
        }

        float r[4];
        r[0] = (max_idx != 0) ? get_res(cp.local_a, points[1].local_a, points[3].local_a, points[2].local_a) : 0;
        r[1] = (max_idx != 1) ? get_res(cp.local_a, points[0].local_a, points[3].local_a, points[2].local_a) : 0;
        r[2] = (max_idx != 2) ? get_res(cp.local_a, points[0].local_a, points[3].local_a, points[1].local_a) : 0;
        r[3] = (max_idx != 3) ? get_res(cp.local_a, points[0].local_a, points[2].local_a, points[1].local_a) : 0;

        int idx = 0; float m = r[0];
        for (int i = 1; i < 4; i++) if (r[i] > m) { m = r[i]; idx = i; }
        return idx;
    }

    RS_INLINE void remove_contact_point(int idx) {
        int last = num_points - 1;
        if (idx != last) points[idx] = points[last];
        num_points--;
    }

    RS_INLINE int add_manifold_point(const ManifoldPoint& cp) {
        if (num_points == MANIFOLD_CACHE_SIZE) {
            int idx = sort_cached(cp);
            points[idx] = cp;
            return idx;
        }
        points[num_points] = cp;
        return num_points++;
    }

    RS_INLINE void refresh_contact_points(const Affine3& trans_a, const Affine3& trans_b) {
        for (int i = num_points - 1; i >= 0; i--) {
            ManifoldPoint& mp = points[i];
            mp.world_a = trans_a.transform_point(mp.local_a);
            mp.world_b = trans_b.transform_point(mp.local_b);
            mp.distance = (mp.world_a - mp.world_b).dot(mp.normal);
        }

        for (int i = num_points - 1; i >= 0; i--) {
            ManifoldPoint& mp = points[i];
            if (!valid_contact_distance(mp)) {
                remove_contact_point(i);
                continue;
            }
            Vec3 projected = mp.world_a - mp.normal * mp.distance;
            Vec3 diff = mp.world_b - projected;
            float d2 = diff.dot(diff);
            if (d2 > breaking_threshold * breaking_threshold) {
                remove_contact_point(i);
            }
        }
    }

    RS_INLINE void clear() { num_points = 0; }
};
