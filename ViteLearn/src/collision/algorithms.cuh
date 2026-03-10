#pragma once

#include "manifold.cuh"
#include "../dynamics/body.cuh"
#include "../sim/constants.cuh"
#include <math.h>

constexpr float FUDGE_FACTOR = 1.05f;
constexpr float FUDGE_2 = 1e-5f;
constexpr float TAU_F = 2.0f * PI_F;

struct Obb {
    Vec3 center;
    Mat3 axis;
    Vec3 extent;

    RS_INLINE Obb() {}
    RS_INLINE Obb(const Vec3& c, const Mat3& a, const Vec3& e) : center(c), axis(a), extent(e) {}
};

struct SatHit {
    float depth;
    Vec3 normal;
    int axis_index;
    bool valid;
};

RS_INLINE float line_closest_approach(const Vec3& pa, const Vec3& ua, const Vec3& pb, const Vec3& ub) {
    Vec3 p = pb - pa;
    float uaub = ua.dot(ub);
    float q1 = ua.dot(p);
    float q2 = -ub.dot(p);
    float d = 1.0f - uaub * uaub;
    return (d <= 0.0001f) ? 0.0f : (uaub * q1 + q2) / d;
}

RS_INLINE int intersect_rect_quad(float hx, float hy, float poly[4][2], float out[16][2]) {
    float q[16][2];
    int qn = 4;
    for (int i = 0; i < 4; i++) { q[i][0] = poly[i][0]; q[i][1] = poly[i][1]; }

    float h[2] = {hx, hy};
    for (int dir = 0; dir < 2; dir++) {
        for (int sign_idx = 0; sign_idx < 2; sign_idx++) {
            float sign = (sign_idx == 0) ? -1.0f : 1.0f;
            float r[16][2];
            int rn = 0;
            float clip_val = sign * h[dir];

            for (int i = 0; i < qn; i++) {
                float cur[2] = {q[i][0], q[i][1]};
                float next[2] = {q[(i+1)%qn][0], q[(i+1)%qn][1]};
                float cur_val = cur[dir];
                float next_val = next[dir];
                bool inside_cur = sign * cur_val <= h[dir];

                if (inside_cur && rn < 16) { r[rn][0] = cur[0]; r[rn][1] = cur[1]; rn++; }

                bool inside_next = sign * next_val <= h[dir];
                if (inside_cur != inside_next && rn < 16) {
                    float denom = next_val - cur_val;
                    float t = (fabsf(denom) < SIMD_EPSILON) ? 0.0f : (clip_val - cur_val) / denom;
                    float p1 = cur[1-dir] + t * (next[1-dir] - cur[1-dir]);
                    if (dir == 0) { r[rn][0] = clip_val; r[rn][1] = p1; }
                    else { r[rn][0] = p1; r[rn][1] = clip_val; }
                    rn++;
                }
            }
            qn = rn;
            for (int i = 0; i < qn; i++) { q[i][0] = r[i][0]; q[i][1] = r[i][1]; }
        }
    }
    for (int i = 0; i < qn; i++) { out[i][0] = q[i][0]; out[i][1] = q[i][1]; }
    return qn;
}

RS_INLINE void cull_points(float p[][2], int n, int i0, int m, int* result) {
    float cx = 0, cy = 0;
    if (n == 1) { cx = p[0][0]; cy = p[0][1]; }
    else if (n == 2) { cx = (p[0][0] + p[1][0]) * 0.5f; cy = (p[0][1] + p[1][1]) * 0.5f; }
    else {
        float a_sum = 0, cx_sum = 0, cy_sum = 0;
        for (int i = 0; i < n - 1; i++) {
            float q = p[i][0] * p[i+1][1] - p[i+1][0] * p[i][1];
            a_sum += q;
            cx_sum += q * (p[i][0] + p[i+1][0]);
            cy_sum += q * (p[i][1] + p[i+1][1]);
        }
        float q_last = p[n-1][0] * p[0][1] - p[0][0] * p[n-1][1];
        float area = a_sum + q_last;
        float inv = (fabsf(area) > SIMD_EPSILON) ? 1.0f / (3.0f * area) : 1e18f;
        cx = inv * (cx_sum + q_last * (p[n-1][0] + p[0][0]));
        cy = inv * (cy_sum + q_last * (p[n-1][1] + p[0][1]));
    }

    float angles[8];
    for (int i = 0; i < n; i++) angles[i] = atan2f(p[i][1] - cy, p[i][0] - cx);

    bool avail[8] = {false};
    for (int i = 1; i < n; i++) avail[i] = true;
    result[0] = i0;

    for (int j = 1; j < m; j++) {
        float target = (float)j * (TAU_F / (float)m) + angles[i0];
        if (target > PI_F) target -= TAU_F;

        int best_idx = i0;
        float best_diff = 1e18f;
        for (int i = 0; i < n; i++) {
            if (avail[i]) {
                float diff = fabsf(angles[i] - target);
                if (diff > PI_F) diff = TAU_F - diff;
                if (diff < best_diff) { best_diff = diff; best_idx = i; }
            }
        }
        avail[best_idx] = false;
        result[j] = best_idx;
    }
}

RS_INLINE SatHit box_box_sat(const Obb& obb1, const Mat3& r1t, const Obb& obb2) {
    SatHit hit; hit.valid = false;
    Vec3 p = obb2.center - obb1.center;
    Vec3 pp = r1t * p;

    Mat3 rij = Mat3(r1t * obb2.axis.x_axis, r1t * obb2.axis.y_axis, r1t * obb2.axis.z_axis);
    Mat3 q = rij.transpose().abs();

    float s = -1e18f;
    float s2;
    Vec3 normal_r;
    Vec3 normal_c;
    bool has_normal_r = false;
    bool invert = false;
    int code = 0;

    Vec3 r1_axes[3] = {obb1.axis.x_axis, obb1.axis.y_axis, obb1.axis.z_axis};

    float expr1, expr2;

    expr1 = pp.x; expr2 = obb1.extent.x + obb2.extent.dot(q.x_axis);
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r1_axes[0]; has_normal_r = true; invert = expr1 < 0; code = 1; }

    expr1 = pp.y; expr2 = obb1.extent.y + obb2.extent.dot(q.y_axis);
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r1_axes[1]; has_normal_r = true; invert = expr1 < 0; code = 2; }

    expr1 = pp.z; expr2 = obb1.extent.z + obb2.extent.dot(q.z_axis);
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r1_axes[2]; has_normal_r = true; invert = expr1 < 0; code = 3; }

    Vec3 r2_axes[3] = {obb2.axis.x_axis, obb2.axis.y_axis, obb2.axis.z_axis};

    expr1 = r2_axes[0].dot(p); expr2 = obb1.extent.dot(q.x_axis) + obb2.extent.x;
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r2_axes[0]; has_normal_r = true; invert = expr1 < 0; code = 4; }

    expr1 = r2_axes[1].dot(p); expr2 = obb1.extent.dot(q.y_axis) + obb2.extent.y;
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r2_axes[1]; has_normal_r = true; invert = expr1 < 0; code = 5; }

    expr1 = r2_axes[2].dot(p); expr2 = obb1.extent.dot(q.z_axis) + obb2.extent.z;
    s2 = fabsf(expr1) - expr2;
    if (s2 > 0) return hit;
    if (s2 > s) { s = s2; normal_r = r2_axes[2]; has_normal_r = true; invert = expr1 < 0; code = 6; }

    q.x_axis = q.x_axis + Vec3::splat(FUDGE_2);
    q.y_axis = q.y_axis + Vec3::splat(FUDGE_2);
    q.z_axis = q.z_axis + Vec3::splat(FUDGE_2);

    Vec3 test_normals[9] = {
        Vec3(0, -rij.z_axis.x, rij.y_axis.x),
        Vec3(0, -rij.z_axis.y, rij.y_axis.y),
        Vec3(0, -rij.z_axis.z, rij.y_axis.z),
        Vec3(rij.z_axis.x, 0, -rij.x_axis.x),
        Vec3(rij.z_axis.y, 0, -rij.x_axis.y),
        Vec3(rij.z_axis.z, 0, -rij.x_axis.z),
        Vec3(-rij.y_axis.x, rij.x_axis.x, 0),
        Vec3(-rij.y_axis.y, rij.x_axis.y, 0),
        Vec3(-rij.y_axis.z, rij.x_axis.z, 0)
    };

    float test_expr1[9] = {
        pp.z * rij.y_axis.x - pp.y * rij.z_axis.x,
        pp.z * rij.y_axis.y - pp.y * rij.z_axis.y,
        pp.z * rij.y_axis.z - pp.y * rij.z_axis.z,
        pp.x * rij.z_axis.x - pp.z * rij.x_axis.x,
        pp.x * rij.z_axis.y - pp.z * rij.x_axis.y,
        pp.x * rij.z_axis.z - pp.z * rij.x_axis.z,
        pp.y * rij.x_axis.x - pp.x * rij.y_axis.x,
        pp.y * rij.x_axis.y - pp.x * rij.y_axis.y,
        pp.y * rij.x_axis.z - pp.x * rij.y_axis.z
    };

    float test_expr2[9] = {
        obb1.extent.y * q.x_axis.z + obb1.extent.z * q.x_axis.y + obb2.extent.y * q.z_axis.x + obb2.extent.z * q.y_axis.x,
        obb1.extent.y * q.y_axis.z + obb1.extent.z * q.y_axis.y + obb2.extent.x * q.z_axis.x + obb2.extent.z * q.x_axis.x,
        obb1.extent.y * q.z_axis.z + obb1.extent.z * q.z_axis.y + obb2.extent.x * q.y_axis.x + obb2.extent.y * q.x_axis.x,
        obb1.extent.x * q.x_axis.z + obb1.extent.z * q.x_axis.x + obb2.extent.y * q.z_axis.y + obb2.extent.z * q.y_axis.y,
        obb1.extent.x * q.y_axis.z + obb1.extent.z * q.y_axis.x + obb2.extent.x * q.z_axis.y + obb2.extent.z * q.x_axis.y,
        obb1.extent.x * q.z_axis.z + obb1.extent.z * q.z_axis.x + obb2.extent.x * q.y_axis.y + obb2.extent.y * q.x_axis.y,
        obb1.extent.x * q.x_axis.y + obb1.extent.y * q.x_axis.x + obb2.extent.y * q.z_axis.z + obb2.extent.z * q.y_axis.z,
        obb1.extent.x * q.y_axis.y + obb1.extent.y * q.y_axis.x + obb2.extent.x * q.z_axis.z + obb2.extent.z * q.x_axis.z,
        obb1.extent.x * q.z_axis.y + obb1.extent.y * q.z_axis.x + obb2.extent.x * q.y_axis.z + obb2.extent.y * q.x_axis.z
    };

    for (int i = 0; i < 9; i++) {
        s2 = fabsf(test_expr1[i]) - test_expr2[i];
        if (s2 > SIMD_EPSILON) return hit;
        float l_sq = test_normals[i].length_squared();
        if (l_sq > SIMD_EPSILON * SIMD_EPSILON) {
            float inv_l = rsqrtf(l_sq);
            s2 *= inv_l;
            if (s2 * FUDGE_FACTOR > s) {
                s = s2;
                has_normal_r = false;
                normal_c = test_normals[i] * inv_l;
                invert = test_expr1[i] < 0;
                code = 7 + i;
            }
        }
    }

    if (code == 0) return hit;

    hit.valid = true;
    hit.normal = has_normal_r ? normal_r : (r1t * normal_c);
    if (invert) hit.normal = hit.normal * -1.0f;
    hit.depth = -s;
    hit.axis_index = code;
    return hit;
}

RS_INLINE void get_face_verts(const Obb& obb, int face_idx, float side_sign, Vec3 out[4]) {
    Vec3 axes[3] = {obb.axis.x_axis, obb.axis.y_axis, obb.axis.z_axis};
    Vec3 u = axes[(face_idx + 1) % 3];
    Vec3 v = axes[(face_idx + 2) % 3];
    float eu = obb.extent[(face_idx + 1) % 3];
    float ev = obb.extent[(face_idx + 2) % 3];
    Vec3 ueu = u * eu;
    Vec3 vev = v * ev;
    Vec3 center = obb.center + axes[face_idx] * obb.extent[face_idx] * side_sign;
    out[0] = center + ueu + vev;
    out[1] = center + ueu - vev;
    out[2] = center - ueu - vev;
    out[3] = center - ueu + vev;
}

RS_INLINE void compute_box_box_contacts(
    const Obb& obb1, const Mat3& r1t,
    const Obb& obb2, const Mat3& r2t,
    const SatHit& hit,
    ManifoldPoint* points, int& num_points
) {
    Vec3 r1t_axes[3] = {r1t.x_axis, r1t.y_axis, r1t.z_axis};
    Vec3 r2t_axes[3] = {r2t.x_axis, r2t.y_axis, r2t.z_axis};

    if (hit.axis_index > 6) {
        Vec3 pa = obb1.center;
        for (int i = 0; i < 3; i++) {
            float sign = (hit.normal.dot(r1t_axes[i]) > 0) ? 1.0f : -1.0f;
            pa = pa + r1t_axes[i] * obb1.extent[i] * sign;
        }

        Vec3 pb = obb2.center;
        for (int i = 0; i < 3; i++) {
            float sign = (hit.normal.dot(r2t_axes[i]) > 0) ? -1.0f : 1.0f;
            pb = pb + r2t_axes[i] * obb2.extent[i] * sign;
        }

        Vec3 ua = r1t_axes[(hit.axis_index - 7) / 3];
        Vec3 ub = r2t_axes[(hit.axis_index - 7) % 3];

        float beta = line_closest_approach(pa, ua, pb, ub);
        Vec3 contact = pb + ub * beta;

        points[num_points].local_a = contact;
        points[num_points].local_b = contact;
        points[num_points].normal = hit.normal * -1.0f;
        points[num_points].distance = -hit.depth;
        num_points++;
        return;
    }

    const Obb* ref_obb = &obb1;
    const Obb* inc_obb = &obb2;
    const Vec3* ref_axes = r1t_axes;
    const Vec3* inc_axes = r2t_axes;

    if (hit.axis_index > 3) {
        ref_obb = &obb2;
        inc_obb = &obb1;
        ref_axes = r2t_axes;
        inc_axes = r1t_axes;
    }

    Vec3 normal_2 = (hit.axis_index <= 3) ? hit.normal : (hit.normal * -1.0f);
    Vec3 nr = inc_obb->axis * normal_2;
    Vec3 anr = nr.abs();

    int lanr = 0, a1 = 1, a2 = 2;
    if (anr.y > anr.x) {
        if (anr.y > anr.z) { lanr = 1; a1 = 0; a2 = 2; }
        else { lanr = 2; a1 = 0; a2 = 1; }
    } else if (anr.x <= anr.z) {
        lanr = 2; a1 = 0; a2 = 1;
    }

    Vec3 center = inc_obb->center - ref_obb->center;
    float sign_lanr = (nr[lanr] < 0) ? 1.0f : -1.0f;
    center = center + inc_axes[lanr] * inc_obb->extent[lanr] * sign_lanr;

    int code_n = (hit.axis_index <= 3) ? hit.axis_index - 1 : hit.axis_index - 4;
    int code1 = (code_n == 0) ? 1 : 0;
    int code2 = (code_n == 2) ? 1 : 2;

    float c1 = center.dot(ref_axes[code1]);
    float c2 = center.dot(ref_axes[code2]);

    float m[4] = {
        ref_axes[code1].dot(inc_axes[a1]),
        ref_axes[code1].dot(inc_axes[a2]),
        ref_axes[code2].dot(inc_axes[a1]),
        ref_axes[code2].dot(inc_axes[a2])
    };

    float k[4] = {
        m[0] * inc_obb->extent[a1],
        m[1] * inc_obb->extent[a1],
        m[2] * inc_obb->extent[a2],
        m[3] * inc_obb->extent[a2]
    };

    float quad[4][2] = {
        {c1 - k[0] - k[2], c2 - k[1] - k[3]},
        {c1 - k[0] + k[2], c2 - k[1] + k[3]},
        {c1 + k[0] + k[2], c2 + k[1] + k[3]},
        {c1 + k[0] - k[2], c2 + k[1] - k[3]}
    };

    float rect[2] = {ref_obb->extent[code1], ref_obb->extent[code2]};
    float ret[16][2];
    int ret_count = intersect_rect_quad(rect[0], rect[1], quad, ret);
    if (ret_count == 0) return;

    float det_val = m[0] * m[3] - m[1] * m[2];
    if (fabsf(det_val) < 1e-12f) return;
    float det1 = 1.0f / det_val;
    m[0] *= det1; m[1] *= det1; m[2] *= det1; m[3] *= det1;

    Vec3 point[8];
    float dep[8];
    int cnum = 0;

    for (int j = 0; j < ret_count && cnum < 8; j++) {
        float k1_p = m[3] * (ret[j][0] - c1) - m[1] * (ret[j][1] - c2);
        float k2_p = -m[2] * (ret[j][0] - c1) + m[0] * (ret[j][1] - c2);
        point[cnum] = center + inc_axes[a1] * k1_p + inc_axes[a2] * k2_p;
        dep[cnum] = ref_obb->extent[code_n] - normal_2.dot(point[cnum]);

        if (dep[cnum] >= 0.0f) {
            ret[cnum][0] = ret[j][0];
            ret[cnum][1] = ret[j][1];
            cnum++;
        }
    }
    if (cnum == 0) return;

    int maxc = (MANIFOLD_CACHE_SIZE < cnum) ? MANIFOLD_CACHE_SIZE : cnum;
    if (maxc < 1) maxc = 1;

    if (cnum <= maxc) {
        for (int i = 0; i < cnum; i++) {
            Vec3 pos = point[i] + ref_obb->center;
            if (hit.axis_index >= 4) pos = pos - hit.normal * dep[i];
            points[num_points].local_a = pos;
            points[num_points].local_b = pos;
            points[num_points].normal = hit.normal * -1.0f;
            points[num_points].distance = -dep[i];
            num_points++;
        }
    } else {
        int i1 = 0;
        float max_dep = dep[0];
        for (int i = 1; i < cnum; i++) {
            if (dep[i] > max_dep) { max_dep = dep[i]; i1 = i; }
        }

        int iret[8];
        cull_points(ret, cnum, i1, maxc, iret);

        for (int j = 0; j < maxc; j++) {
            int idx = iret[j];
            Vec3 pos = point[idx] + ref_obb->center;
            if (hit.axis_index >= 4) pos = pos - hit.normal * dep[idx];
            points[num_points].local_a = pos;
            points[num_points].local_b = pos;
            points[num_points].normal = hit.normal * -1.0f;
            points[num_points].distance = -dep[idx];
            num_points++;
        }
    }
}

struct BoxTriHit {
    float depth;
    Vec3 normal;
    int axis_index;
    bool neg_axis;
    bool valid;
};

RS_INLINE void project_triangle(const Vec3 tri[3], const Vec3& axis, float& out_min, float& out_max) {
    float p0 = tri[0].dot(axis);
    float p1 = tri[1].dot(axis);
    float p2 = tri[2].dot(axis);
    out_min = fminf(fminf(p0, p1), p2);
    out_max = fmaxf(fmaxf(p0, p1), p2);
}

RS_INLINE float project_box_radius(const Vec3& extent, const Vec3& axis) {
    return extent.dot(axis.abs());
}

RS_INLINE BoxTriHit aabb_triangle_sat(
    const Vec3& extent,
    const Vec3 tri_local[3],
    const Vec3& tri_normal,
    const Vec3 tri_edges[3],
    float tri_normal_depth,
    bool tri_normal_neg_axis
) {
    BoxTriHit hit;
    hit.valid = false;

    float min_depth = -tri_normal_depth;
    Vec3 min_axis = tri_normal_neg_axis ? (tri_normal * -1.0f) : tri_normal;
    int min_axis_index = 0;
    bool min_neg_axis = tri_normal_neg_axis;

    const Vec3 IDENT_AXES[3] = {Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)};
    int axis_index = 0;

    for (int i = 0; i < 3; i++) {
        axis_index++;
        Vec3 obb_axis = IDENT_AXES[i];
        float r_obb = project_box_radius(extent, obb_axis);
        float tri_min, tri_max;
        project_triangle(tri_local, obb_axis, tri_min, tri_max);

        if (tri_max < -r_obb || tri_min > r_obb) {
            return hit;
        }

        float overlap_neg = tri_max + r_obb;
        float overlap_pos = r_obb - tri_min;
        bool neg_axis = overlap_neg < overlap_pos;
        float depth = neg_axis ? overlap_neg : overlap_pos;
        Vec3 normal = neg_axis ? (obb_axis * -1.0f) : obb_axis;

        if (depth < min_depth) {
            min_depth = depth;
            min_axis = normal;
            min_axis_index = axis_index;
            min_neg_axis = neg_axis;
        }
    }

    for (int i = 0; i < 3; i++) {
        Vec3 obb_axis = IDENT_AXES[i];
        for (int j = 0; j < 3; j++) {
            axis_index++;
            Vec3 cross = obb_axis.cross(tri_edges[j]);
            float len_sq = cross.length_squared();
            if (len_sq < SIMD_EPSILON * SIMD_EPSILON) continue;

            Vec3 axis = cross * rsqrtf(len_sq);
            float r_obb = project_box_radius(extent, axis);
            float tri_min, tri_max;
            project_triangle(tri_local, axis, tri_min, tri_max);

            if (tri_max < -r_obb || tri_min > r_obb) {
                return hit;
            }

            float overlap_neg = tri_max + r_obb;
            float overlap_pos = r_obb - tri_min;
            bool neg_axis = overlap_neg < overlap_pos;
            float depth = neg_axis ? overlap_neg : overlap_pos;
            Vec3 normal = neg_axis ? (axis * -1.0f) : axis;

            if (depth < min_depth) {
                min_depth = depth;
                min_axis = normal;
                min_axis_index = axis_index;
                min_neg_axis = neg_axis;
            }
        }
    }

    hit.valid = true;
    hit.depth = min_depth;
    hit.normal = min_axis;
    hit.axis_index = min_axis_index;
    hit.neg_axis = min_neg_axis;
    return hit;
}

RS_INLINE Vec3 closest_point_on_segment(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    float t = (p - a).dot(ab) / ab.dot(ab);
    t = fmaxf(0.0f, fminf(1.0f, t));
    return a + ab * t;
}

RS_INLINE Vec3 project_point_on_inf_line(const Vec3& point, const Vec3& line_a, const Vec3& line_b) {
    Vec3 line_vec = line_b - line_a;
    float len_sq = line_vec.length_squared();
    if (len_sq < SIMD_EPSILON) return line_a;
    float t = (point - line_a).dot(line_vec) / len_sq;
    return line_a + line_vec * t;
}

RS_INLINE Vec3 closest_points_between_segments(
    const Vec3& seg_a, const Vec3& seg_b,
    const Vec3& seg_c, const Vec3& seg_d
) {
    Vec3 in_plane_a = project_point_on_inf_line(seg_a, seg_c, seg_d);
    Vec3 in_plane_b = project_point_on_inf_line(seg_b, seg_c, seg_d);
    Vec3 in_plane_ba = in_plane_b - in_plane_a;

    float denom = in_plane_ba.length_squared();
    float t = (denom > SIMD_EPSILON)
        ? (seg_c - in_plane_a).dot(in_plane_ba) / denom
        : 0.0f;
    t = fmaxf(0.0f, fminf(1.0f, t));

    Vec3 on_ab = seg_a + (seg_b - seg_a) * t;
    Vec3 on_cd = closest_point_on_segment(on_ab, seg_c, seg_d);
    Vec3 on_ab_refined = closest_point_on_segment(on_cd, seg_a, seg_b);

    return (on_ab_refined + on_cd) * 0.5f;
}

RS_INLINE Vec3 closest_point_on_triangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return a;

    Vec3 bp = p - b;
    float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    Vec3 cp = p - c;
    float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

RS_INLINE bool box_triangle_collision(
    const Affine3& box_trans,
    const Vec3& box_half_extents,
    const Vec3 tri_world[3],
    const Vec3& tri_normal_world,
    float contact_threshold,
    Vec3& out_normal,
    Vec3& out_point,
    float& out_depth
) {
    Affine3 box_inv = box_trans.transpose();
    Vec3 tri_local[3];
    for (int i = 0; i < 3; i++) {
        tri_local[i] = box_inv.transform_point(tri_world[i]);
    }

    Vec3 tri_normal_local = box_inv.matrix3 * tri_normal_world;

    Vec3 tri_edges[3];
    tri_edges[0] = tri_local[1] - tri_local[0];
    tri_edges[1] = tri_local[2] - tri_local[1];
    tri_edges[2] = tri_local[0] - tri_local[2];

    float plane_dist = tri_normal_local.dot(tri_local[0]);
    float proj_radius = box_half_extents.dot(tri_normal_local.abs());
    if (fabsf(plane_dist) > proj_radius) return false;

    float penetration = proj_radius - fabsf(plane_dist);
    bool tri_normal_neg_axis = (plane_dist > 0);
    float tri_normal_depth = -penetration;

    BoxTriHit hit = aabb_triangle_sat(box_half_extents, tri_local, tri_normal_local, tri_edges, tri_normal_depth, tri_normal_neg_axis);
    if (!hit.valid) return false;

    out_normal = box_trans.matrix3 * hit.normal;
    if (hit.axis_index != 0) out_normal = out_normal * -1.0f;

    if (hit.axis_index == 0) {
        Vec3 tri_centroid = (tri_local[0] + tri_local[1] + tri_local[2]) * (1.0f / 3.0f);
        float plane_dist = hit.normal.dot(tri_local[0]);

        Vec3 best_pt = Vec3::zero();
        float min_dist_sq = 1e30f;
        bool found_corner = false;

        for (int ci = 0; ci < 8; ci++) {
            Vec3 corner = Vec3(
                (ci & 1) ? box_half_extents.x : -box_half_extents.x,
                (ci & 2) ? box_half_extents.y : -box_half_extents.y,
                (ci & 4) ? box_half_extents.z : -box_half_extents.z
            );
            float d2p = hit.normal.dot(corner) - plane_dist;
            Vec3 proj = corner - hit.normal * d2p;
            float dsq = (proj - tri_centroid).length_squared();
            if (dsq < min_dist_sq && fabsf(d2p) < 0.1f) {
                min_dist_sq = dsq;
                best_pt = proj;
                found_corner = true;
            }
        }

        if (!found_corner) {
            Vec3 neg_n = hit.normal * -1.0f;
            Vec3 support_local = Vec3(
                neg_n.x >= 0 ? box_half_extents.x : -box_half_extents.x,
                neg_n.y >= 0 ? box_half_extents.y : -box_half_extents.y,
                neg_n.z >= 0 ? box_half_extents.z : -box_half_extents.z
            );
            float d2p = hit.normal.dot(support_local) - plane_dist;
            best_pt = support_local - hit.normal * d2p;
        }

        out_point = box_trans.matrix3 * best_pt + box_trans.translation;
    } else if (hit.axis_index >= 1 && hit.axis_index <= 3) {
        int face_idx = hit.axis_index - 1;
        float face_coord = (hit.normal[face_idx] > 0.0f) ? box_half_extents[face_idx] : -box_half_extents[face_idx];

        Vec3 candidates[6];
        int num_cands = 0;

        for (int vi = 0; vi < 3; vi++) {
            if (fabsf(tri_local[vi][face_idx] - face_coord) < 0.05f) {
                Vec3 av = Vec3(fabsf(tri_local[vi].x), fabsf(tri_local[vi].y), fabsf(tri_local[vi].z));
                float mx = fmaxf(av.x - box_half_extents.x, fmaxf(av.y - box_half_extents.y, av.z - box_half_extents.z));
                if (mx < 0.05f) {
                    candidates[num_cands++] = tri_local[vi];
                }
            }
        }

        for (int ei = 0; ei < 3; ei++) {
            Vec3 s = tri_local[ei];
            Vec3 e = tri_local[(ei + 1) % 3];
            float da = s[face_idx] - face_coord;
            float db = e[face_idx] - face_coord;
            if (da * db < 0.0f) {
                float t = da / (da - db);
                Vec3 isect = s + (e - s) * t;
                Vec3 ai = Vec3(fabsf(isect.x), fabsf(isect.y), fabsf(isect.z));
                float mx = fmaxf(ai.x - box_half_extents.x, fmaxf(ai.y - box_half_extents.y, ai.z - box_half_extents.z));
                if (mx < 0.05f && num_cands < 6) {
                    candidates[num_cands++] = isect;
                }
            }
        }

        if (num_cands == 0) {
            int deepest_idx = 0;
            float max_dot = hit.normal.dot(tri_local[0]);
            for (int i = 1; i < 3; i++) {
                float d = hit.normal.dot(tri_local[i]);
                if (d > max_dot) { max_dot = d; deepest_idx = i; }
            }
            out_point = box_trans.matrix3 * tri_local[deepest_idx] + box_trans.translation;
        } else {
            Vec3 avg = candidates[0];
            for (int i = 1; i < num_cands; i++) avg = avg + candidates[i];
            avg = avg * (1.0f / (float)num_cands);
            out_point = box_trans.matrix3 * avg + box_trans.translation;
        }
    } else {
        int box_axis_idx = (hit.axis_index - 4) / 3;
        int tri_edge_idx = (hit.axis_index - 4) % 3;

        Vec3 edge_pos = Vec3(
            (hit.normal.x >= 0) ? box_half_extents.x : -box_half_extents.x,
            (hit.normal.y >= 0) ? box_half_extents.y : -box_half_extents.y,
            (hit.normal.z >= 0) ? box_half_extents.z : -box_half_extents.z
        );
        Vec3 edge_start = edge_pos;
        Vec3 edge_end = edge_pos;

        if (box_axis_idx == 0) { edge_start.x = box_half_extents.x; edge_end.x = -box_half_extents.x; }
        else if (box_axis_idx == 1) { edge_start.y = box_half_extents.y; edge_end.y = -box_half_extents.y; }
        else { edge_start.z = box_half_extents.z; edge_end.z = -box_half_extents.z; }

        Vec3 local_contact = closest_points_between_segments(
            edge_start, edge_end,
            tri_local[tri_edge_idx], tri_local[(tri_edge_idx + 1) % 3]
        );

        out_point = box_trans.matrix3 * local_contact + box_trans.translation;
    }

    out_depth = -hit.depth;
    return true;
}

RS_INLINE bool sphere_obb_collision(
    const Vec3& sphere_pos, float sphere_radius,
    const Affine3& obb_trans, const Vec3& obb_half_extents_without_margin,
    float box_margin,
    Vec3& out_normal, Vec3& out_point, float& out_depth
) {
    Vec3 local_sphere = obb_trans.inv_xform(sphere_pos);
    Vec3 box_min = obb_half_extents_without_margin * -1.0f;
    Vec3 box_max = obb_half_extents_without_margin;

    Vec3 closest = Vec3(
        fmaxf(box_min.x, fminf(local_sphere.x, box_max.x)),
        fmaxf(box_min.y, fminf(local_sphere.y, box_max.y)),
        fmaxf(box_min.z, fminf(local_sphere.z, box_max.z))
    );

    Vec3 delta = local_sphere - closest;
    float dist_sq = delta.length_squared();
    float collision_threshold = sphere_radius + box_margin;

    if (dist_sq >= collision_threshold * collision_threshold) return false;

    Vec3 local_normal;

    if (dist_sq > SIMD_EPSILON) {
        float inv_dist = rsqrtf(dist_sq);
        local_normal = delta * inv_dist;
        out_depth = dist_sq * inv_dist - (sphere_radius + box_margin);
    } else {
        local_normal = Vec3(1, 0, 0);
        out_depth = -(sphere_radius + box_margin);
    }

    out_normal = obb_trans.matrix3 * local_normal;
    out_point = obb_trans.transform_point(closest) + out_normal * box_margin;
    return true;
}
