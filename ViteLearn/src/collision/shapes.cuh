#pragma once

#include "../math/geo.cuh"

constexpr float TAU_F_SHAPES = 2.0f * 3.14159265358979323846f;
constexpr float CONVEX_DISTANCE_MARGIN = 0.04f;
constexpr float CONTACT_BREAKING_THRESHOLD = 0.02f;
constexpr float SPHERE_RADIUS_MARGIN = 0.08f;

struct SphereShape {
    float radius;

    RS_INLINE SphereShape() : radius(0) {}
    RS_INLINE SphereShape(float r) : radius(r) {}

    RS_INLINE Aabb get_aabb(const Affine3& t) const {
        Vec3 center = t.translation;
        float margin = radius + SPHERE_RADIUS_MARGIN;
        Vec3 extent = Vec3::splat(margin);
        return Aabb(center - extent, center + extent);
    }

    RS_INLINE Vec3 support(const Vec3& dir) const {
        return dir.normalized() * radius;
    }

    RS_INLINE float get_angular_motion_disc() const {
        return radius + SPHERE_RADIUS_MARGIN;
    }

    RS_INLINE float get_contact_breaking_threshold(float base_threshold) const {
        return get_angular_motion_disc() * base_threshold;
    }
};

struct BoxShape {
    Vec3 implicit_shape_dimensions;
    float collision_margin;

    RS_INLINE BoxShape() : implicit_shape_dimensions(Vec3::zero()), collision_margin(0) {}
    RS_INLINE BoxShape(const Vec3& half_ext) {
        implicit_shape_dimensions = half_ext - Vec3::splat(CONVEX_DISTANCE_MARGIN);
        float safe_margin = 0.1f * fminf(fminf(half_ext.x, half_ext.y), half_ext.z);
        collision_margin = fminf(safe_margin, CONVEX_DISTANCE_MARGIN);
    }

    RS_INLINE Vec3 get_half_extents() const {
        return implicit_shape_dimensions;
    }

    RS_INLINE Vec3 get_half_extents_with_margin() const {
        return implicit_shape_dimensions + Vec3::splat(collision_margin);
    }

    RS_INLINE Aabb get_aabb(const Affine3& t) const {
        Vec3 he = get_half_extents_with_margin();
        Mat3 abs_b = t.matrix3.abs();
        Vec3 extent = abs_b * he;
        return Aabb(t.translation - extent, t.translation + extent);
    }

    RS_INLINE Vec3 support(const Vec3& dir) const {
        Vec3 he = get_half_extents();
        return Vec3(dir.x >= 0 ? he.x : -he.x, dir.y >= 0 ? he.y : -he.y, dir.z >= 0 ? he.z : -he.z);
    }

    RS_INLINE Vec3 calculate_local_inertia(float mass) const {
        Vec3 he = get_half_extents();
        float lx = 2.0f * he.x, ly = 2.0f * he.y, lz = 2.0f * he.z;
        float f = mass / 12.0f;
        return Vec3(f * (ly*ly + lz*lz), f * (lx*lx + lz*lz), f * (lx*lx + ly*ly));
    }

    RS_INLINE float get_angular_motion_disc() const {
        Vec3 he = get_half_extents();
        return sqrtf(he.x * he.x + he.y * he.y + he.z * he.z);
    }

    RS_INLINE float get_contact_breaking_threshold(float base_threshold) const {
        return get_angular_motion_disc() * base_threshold;
    }
};

struct TriangleShape {
    Vec3 verts[3];
    Vec3 edges[3];
    Vec3 normal;
    float normal_length;

    RS_INLINE TriangleShape() : normal_length(0) {}

    RS_INLINE void init(const Vec3& p0, const Vec3& p1, const Vec3& p2) {
        verts[0] = p0; verts[1] = p1; verts[2] = p2;
        edges[0] = p1 - p0; edges[1] = p2 - p1; edges[2] = p0 - p2;
        Vec3 cross = edges[0].cross(edges[2] * -1.0f);
        normal_length = cross.length();
        normal = normal_length > 0 ? cross * (1.0f / normal_length) : Vec3::zero();
    }

    RS_INLINE bool face_contains(const Vec3& n, const Vec3 d[3]) const {
        return edges[0].cross(d[0]).dot(n) >= 0 &&
               edges[1].cross(d[1]).dot(n) >= 0 &&
               edges[2].cross(d[2]).dot(n) >= 0;
    }

    RS_INLINE Vec3 closest_point(const Vec3 d[3]) const {
        Vec3 ab = edges[0], ac = edges[2] * -1.0f;
        float d1 = ab.dot(d[0]), d2 = ac.dot(d[0]);
        if (d1 <= 0 && d2 <= 0) return verts[0];

        float d3 = ab.dot(d[1]), d4 = ac.dot(d[1]);
        if (d3 >= 0 && d4 <= d3) return verts[1];

        float d5 = ab.dot(d[2]), d6 = ac.dot(d[2]);
        if (d6 >= 0 && d5 <= d6) return verts[2];

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) return verts[0] + ab * (d1 / (d1 - d3));

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) return verts[0] + ac * (d2 / (d2 - d6));

        float va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
            return verts[1] + edges[1] * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

        float sum = va + vb + vc;
        if (fabsf(sum) < 1e-12f) return verts[0];
        float denom = 1.0f / sum;
        return verts[0] + ab * (vb * denom) + ac * (vc * denom);
    }

    RS_INLINE bool intersect_ray(const Vec3& from, const Vec3& to, float& t_max, Vec3& out_normal, Vec3& out_point) const {
        constexpr float EDGE_TOLERANCE = -0.0001f;

        float dist = verts[0].dot(normal);
        float dist_a = normal.dot(from) - dist;
        float dist_b = normal.dot(to) - dist;
        if (dist_a * dist_b >= 0.0f) return false;

        float proj_length = dist_a - dist_b;
        float t = dist_a / proj_length;
        if (t >= t_max) return false;

        Vec3 point = from + (to - from) * t;
        Vec3 v0p = verts[0] - point;
        Vec3 v1p = verts[1] - point;
        Vec3 cp0 = v0p.cross(v1p);
        if (cp0.dot(normal) < EDGE_TOLERANCE) return false;

        Vec3 v2p = verts[2] - point;
        Vec3 cp1 = v1p.cross(v2p);
        if (cp1.dot(normal) < EDGE_TOLERANCE) return false;

        Vec3 cp2 = v2p.cross(v0p);
        if (cp2.dot(normal) < EDGE_TOLERANCE) return false;

        t_max = t;
        out_normal = (dist_a <= 0.0f) ? normal * -1.0f : normal;
        out_point = point;
        return true;
    }

    RS_INLINE bool intersect_sphere(const Vec3& center, float radius, float threshold, Vec3& out_normal, Vec3& out_point, float& out_depth) const {
        Vec3 p1_to_centre = center - verts[0];
        float distance_from_plane = p1_to_centre.dot(normal);

        Vec3 tri_normal = normal;
        if (distance_from_plane < 0.0f) {
            distance_from_plane = -distance_from_plane;
            tri_normal = tri_normal * -1.0f;
        }

        float radius_with_threshold = radius + threshold;
        if (distance_from_plane >= radius_with_threshold) return false;

        Vec3 d[3] = { p1_to_centre, center - verts[1], center - verts[2] };
        Vec3 contact_point;
        bool has_contact = false;

        if (face_contains(tri_normal, d)) {
            has_contact = true;
            contact_point = center - tri_normal * distance_from_plane;
        } else {
            contact_point = closest_point(d);
            float contact_dist_sq = (contact_point - center).length_squared();
            if (contact_dist_sq < radius_with_threshold * radius_with_threshold) {
                has_contact = true;
            }
        }

        if (!has_contact) return false;

        Vec3 contact_to_centre = center - contact_point;
        float dist_sq = contact_to_centre.length_squared();

        if (dist_sq >= radius_with_threshold * radius_with_threshold) return false;

        if (dist_sq > SIMD_EPSILON) {
            float inv_dist = rsqrtf(dist_sq);
            out_normal = contact_to_centre * inv_dist;
            out_depth = -(radius - dist_sq * inv_dist);
        } else {
            out_normal = tri_normal;
            out_depth = -radius;
        }
        out_point = contact_point;
        return true;
    }
};

enum TriInfoFlag {
    TRI_V0V1_CONVEX = 1,
    TRI_V1V2_CONVEX = 2,
    TRI_V2V0_CONVEX = 4,
    TRI_V0V1_SWAP_NORMAL_B = 8,
    TRI_V1V2_SWAP_NORMAL_B = 16,
    TRI_V2V0_SWAP_NORMAL_B = 32
};

constexpr float CONVEX_EPSILON = 0.0f;
constexpr float PLANAR_EPSILON = 0.0001f;
constexpr float EQUAL_VERTEX_THRESHOLD = 0.0001f * 0.0001f;
constexpr float EDGE_DISTANCE_THRESHOLD = 0.1f;
constexpr float MAX_EDGE_ANGLE_THRESHOLD = TAU_F_SHAPES;

struct TriangleInfo {
    unsigned char flags;
    float edge_v0v1_angle;
    float edge_v1v2_angle;
    float edge_v2v0_angle;

    RS_INLINE TriangleInfo() : flags(0), edge_v0v1_angle(TAU_F_SHAPES), edge_v1v2_angle(TAU_F_SHAPES), edge_v2v0_angle(TAU_F_SHAPES) {}
};

enum ShapeType { SHAPE_SPHERE, SHAPE_BOX, SHAPE_MESH };

struct CollisionShape {
    ShapeType type;
    union {
        SphereShape sphere;
        BoxShape box;
    };

    RS_INLINE CollisionShape() : type(SHAPE_SPHERE) { sphere = SphereShape(); }

    RS_INLINE Aabb get_aabb(const Affine3& t) const {
        if (type == SHAPE_SPHERE) return sphere.get_aabb(t);
        if (type == SHAPE_BOX) return box.get_aabb(t);
        return Aabb();
    }

    RS_INLINE float get_angular_motion_disc() const {
        if (type == SHAPE_SPHERE) return sphere.get_angular_motion_disc();
        if (type == SHAPE_BOX) return box.get_angular_motion_disc();
        return 1.0f;
    }

    RS_INLINE float get_contact_breaking_threshold(float base_threshold) const {
        return get_angular_motion_disc() * base_threshold;
    }
};
