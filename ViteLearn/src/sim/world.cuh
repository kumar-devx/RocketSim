#pragma once

#include "car.cuh"
#include "ball.cuh"
#include "boost.cuh"
#include "../dynamics/constraint.cuh"
#include "../collision/bvh.cuh"
#include "../collision/algorithms.cuh"
#include <cstdio>
#include <cstdint>
#include <cstring>

constexpr int MAX_BODIES = 10;
constexpr int MAX_MANIFOLDS = 64;
constexpr int MAX_TRIANGLES_PER_MESH = 1024;
constexpr int NUM_ARENA_MESHES = 16;

constexpr int USER_TYPE_NONE = 0;
constexpr int USER_TYPE_BALL = 1;
constexpr int USER_TYPE_CAR = 2;

constexpr int PLANE_FLOOR = -2;
constexpr int PLANE_CEILING = -3;
constexpr int PLANE_WALL_X_NEG = -4;
constexpr int PLANE_WALL_X_POS = -5;
constexpr int MESH_ID_BASE = -10;

struct BallContactDebug {
    Vec3 normal;
    float depth;
    int tri_idx;
    bool has_contact;

    RS_INLINE BallContactDebug() : normal(Vec3::zero()), depth(0), tri_idx(-1), has_contact(false) {}

    RS_INLINE void reset() {
        normal = Vec3::zero();
        depth = 0;
        tri_idx = -1;
        has_contact = false;
    }
};

struct BallCarCollisionState {
    Vec3 ball_pos;
    Vec3 ball_vel;
    Vec3 car_pos;
    Vec3 car_vel;
    Vec3 car_forward;
    int car_user_id;
    bool has_collision;

    RS_INLINE BallCarCollisionState() : ball_pos(Vec3::zero()), ball_vel(Vec3::zero()),
        car_pos(Vec3::zero()), car_vel(Vec3::zero()), car_forward(Vec3::zero()),
        car_user_id(-1), has_collision(false) {}

    RS_INLINE void reset() {
        has_collision = false;
        car_user_id = -1;
    }
};

RS_INLINE float get_edge_angle(const Vec3& edge_a, const Vec3& normal_a, const Vec3& normal_b) {
    return atan2f(normal_b.dot(edge_a), normal_b.dot(normal_a));
}

RS_INLINE Vec3 nearest_point_on_segment(const Vec3& point, const Vec3& line0, const Vec3& line1) {
    Vec3 line_delta = line1 - line0;
    float len_sq = line_delta.length_squared();
    if (len_sq < SIMD_EPSILON * SIMD_EPSILON) return line0;
    float delta = (point - line0).dot(line_delta) / len_sq;
    delta = fmaxf(0.0f, fminf(1.0f, delta));
    return line0 + line_delta * delta;
}

RS_INLINE Vec3 quat_rotate(const Quat& q, const Vec3& v) {
    Vec3 u(q.x, q.y, q.z);
    float s = q.w;
    return u * 2.0f * u.dot(v) + v * (s * s - u.dot(u)) + u.cross(v) * 2.0f * s;
}

struct TriangleMesh {
    TriangleShape triangles[MAX_TRIANGLES_PER_MESH];
    TriangleInfo tri_info[MAX_TRIANGLES_PER_MESH];
    Aabb tri_aabbs[MAX_TRIANGLES_PER_MESH];
    int num_tris;
    Bvh bvh;

    RS_INLINE TriangleMesh() : num_tris(0) {}

    RS_INLINE void add_triangle(const Vec3& p0, const Vec3& p1, const Vec3& p2) {
        if (num_tris >= MAX_TRIANGLES_PER_MESH) return;
        triangles[num_tris].init(p0, p1, p2);
        tri_aabbs[num_tris] = Aabb::from_triangle(p0, p1, p2);
        tri_info[num_tris] = TriangleInfo();
        num_tris++;
    }

    void build_bvh() {
        if (num_tris == 0) return;

        BvhNode* leaf_nodes = new BvhNode[num_tris];
        Aabb root_aabb;

        for (int i = 0; i < num_tris; i++) {
            leaf_nodes[i].aabb = update_triangle_aabb(tri_aabbs[i]);
            leaf_nodes[i].node_type = NODE_LEAF;
            leaf_nodes[i].triangle_index = i;
            leaf_nodes[i].escape_index = 1;
            root_aabb = root_aabb.merge(leaf_nodes[i].aabb);
        }

        bvh.build(leaf_nodes, num_tris, root_aabb);
        delete[] leaf_nodes;
    }

    void generate_edge_info() {
        for (int i = 0; i < num_tris; i++) {
            TriangleShape& tri_a = triangles[i];
            if (tri_a.normal_length < EQUAL_VERTEX_THRESHOLD) continue;

            for (int j = 0; j < num_tris; j++) {
                if (i == j) continue;
                TriangleShape& tri_b = triangles[j];
                if (tri_b.normal_length < EQUAL_VERTEX_THRESHOLD) continue;

                int shared_a[2], shared_b[2];
                int num_shared = 0;

                for (int va = 0; va < 3 && num_shared < 2; va++) {
                    for (int vb = 0; vb < 3 && num_shared < 2; vb++) {
                        if ((tri_a.verts[va] - tri_b.verts[vb]).length_squared() < EQUAL_VERTEX_THRESHOLD) {
                            shared_a[num_shared] = va;
                            shared_b[num_shared] = vb;
                            num_shared++;
                        }
                    }
                }

                if (num_shared == 2) {
                    if (shared_a[0] == 0 && shared_a[1] == 2) {
                        int tmp = shared_a[0]; shared_a[0] = shared_a[1]; shared_a[1] = tmp;
                        tmp = shared_b[0]; shared_b[0] = shared_b[1]; shared_b[1] = tmp;
                    }

                    int sum_a = shared_a[0] + shared_a[1];
                    int other_a = 3 - sum_a;
                    int other_b = 3 - (shared_b[0] + shared_b[1]);

                    Vec3 edge = (tri_a.verts[shared_a[1]] - tri_a.verts[shared_a[0]]).normalized();

                    Vec3 edge_cross_a = edge.cross(tri_a.normal).normalized();
                    Vec3 tmp_a = tri_a.verts[other_a] - tri_a.verts[shared_a[0]];
                    if (edge_cross_a.dot(tmp_a) < 0) edge_cross_a = edge_cross_a * -1.0f;

                    Vec3 edge_cross_b = edge.cross(tri_b.normal).normalized();
                    Vec3 tmp_b = tri_b.verts[other_b] - tri_b.verts[shared_b[0]];
                    if (edge_cross_b.dot(tmp_b) < 0) edge_cross_b = edge_cross_b * -1.0f;

                    Vec3 calc_edge = edge_cross_a.cross(edge_cross_b);
                    float len2 = calc_edge.length_squared();

                    float corrected_angle = 0.0f;
                    bool is_convex = false;

                    if (len2 >= PLANAR_EPSILON) {
                        Vec3 calc_edge_n = calc_edge.normalized();
                        Vec3 calc_normal_a = calc_edge_n.cross(edge_cross_a).normalized();
                        float angle2 = get_edge_angle(calc_normal_a, edge_cross_a, edge_cross_b);
                        float ang4 = PI_F - angle2;
                        float dot_a = tri_a.normal.dot(edge_cross_b);
                        is_convex = dot_a < 0.0f;
                        corrected_angle = is_convex ? ang4 : -ang4;
                    }

                    TriangleInfo& info = tri_info[i];
                    int edge_idx = sum_a - 1;
                    Vec3 edge_neg = tri_a.edges[edge_idx == 1 ? 2 : edge_idx == 2 ? 1 : 0] * -1.0f;
                    Quat orn = quat_from_axis_angle(edge_neg.normalized(), -corrected_angle);
                    Vec3 computed_normal_b = quat_rotate(orn, tri_a.normal);
                    bool swap_normal = computed_normal_b.dot(tri_b.normal) < 0.0f;

                    if (sum_a == 1) {
                        info.edge_v0v1_angle = -corrected_angle;
                        if (is_convex) info.flags |= TRI_V0V1_CONVEX;
                        if (swap_normal) info.flags |= TRI_V0V1_SWAP_NORMAL_B;
                    } else if (sum_a == 2) {
                        info.edge_v2v0_angle = -corrected_angle;
                        if (is_convex) info.flags |= TRI_V2V0_CONVEX;
                        if (swap_normal) info.flags |= TRI_V2V0_SWAP_NORMAL_B;
                    } else {
                        info.edge_v1v2_angle = -corrected_angle;
                        if (is_convex) info.flags |= TRI_V1V2_CONVEX;
                        if (swap_normal) info.flags |= TRI_V1V2_SWAP_NORMAL_B;
                    }
                }
            }
        }
    }

    RS_INLINE void adjust_contact_for_internal_edge(int tri_idx, Vec3& contact_normal, Vec3& contact_point, const Vec3& pos_on_a, float distance) const {
        const TriangleInfo& info = tri_info[tri_idx];
        const TriangleShape& tri = triangles[tri_idx];

        float dists[3];
        int best_edge = -1;
        float best_dist = 1e18f;

        if (fabsf(info.edge_v0v1_angle) < MAX_EDGE_ANGLE_THRESHOLD) {
            Vec3 nearest = nearest_point_on_segment(contact_point, tri.verts[0], tri.verts[1]);
            dists[0] = (contact_point - nearest).length();
            if (dists[0] < best_dist) { best_edge = 0; best_dist = dists[0]; }
        }
        if (fabsf(info.edge_v1v2_angle) < MAX_EDGE_ANGLE_THRESHOLD) {
            Vec3 nearest = nearest_point_on_segment(contact_point, tri.verts[1], tri.verts[2]);
            dists[1] = (contact_point - nearest).length();
            if (dists[1] < best_dist) { best_edge = 1; best_dist = dists[1]; }
        }
        if (fabsf(info.edge_v2v0_angle) < MAX_EDGE_ANGLE_THRESHOLD) {
            Vec3 nearest = nearest_point_on_segment(contact_point, tri.verts[2], tri.verts[0]);
            dists[2] = (contact_point - nearest).length();
            if (dists[2] < best_dist) { best_edge = 2; best_dist = dists[2]; }
        }

        if (best_edge < 0) {
            return;
        }
        if (best_dist >= EDGE_DISTANCE_THRESHOLD) {
            return;
        }

        float edge_angle;
        bool is_convex;
        bool swap_normal;
        Vec3 edge;

        if (best_edge == 0) {
            edge_angle = info.edge_v0v1_angle;
            is_convex = (info.flags & TRI_V0V1_CONVEX) != 0;
            swap_normal = (info.flags & TRI_V0V1_SWAP_NORMAL_B) != 0;
            edge = tri.edges[0] * -1.0f;
        } else if (best_edge == 1) {
            edge_angle = info.edge_v1v2_angle;
            is_convex = (info.flags & TRI_V1V2_CONVEX) != 0;
            swap_normal = (info.flags & TRI_V1V2_SWAP_NORMAL_B) != 0;
            edge = tri.edges[1] * -1.0f;
        } else {
            edge_angle = info.edge_v2v0_angle;
            is_convex = (info.flags & TRI_V2V0_CONVEX) != 0;
            swap_normal = (info.flags & TRI_V2V0_SWAP_NORMAL_B) != 0;
            edge = tri.edges[2] * -1.0f;
        }

        if (edge_angle == 0.0f) {
            if (contact_normal.dot(tri.normal) >= 0) {
                contact_normal = tri.normal;
                contact_point = pos_on_a - contact_normal * distance;
            }
            return;
        }

        float swap_factor = is_convex ? 1.0f : -1.0f;
        Vec3 n_a = tri.normal * swap_factor;
        Quat orn = quat_from_axis_angle(edge.normalized(), edge_angle);
        Vec3 computed_normal_b = quat_rotate(orn, tri.normal);
        if (swap_normal) computed_normal_b = computed_normal_b * -1.0f;
        Vec3 n_b = computed_normal_b * swap_factor;

        float n_dot_a = contact_normal.dot(n_a);
        float n_dot_b = contact_normal.dot(n_b);
        bool back_facing = n_dot_a < CONVEX_EPSILON && n_dot_b < CONVEX_EPSILON;

        if (!back_facing) {
            Vec3 edge_cross = edge.cross(tri.normal * swap_factor).normalized();
            float cur_angle = get_edge_angle(edge_cross, tri.normal * swap_factor, contact_normal);

            bool needs_clamp = (edge_angle < 0 && cur_angle < edge_angle) ||
                               (edge_angle >= 0 && cur_angle > edge_angle);

            if (needs_clamp) {
                float diff_angle = edge_angle - cur_angle;
                Quat rot = quat_from_axis_angle(edge.normalized(), diff_angle);
                Vec3 clamped = quat_rotate(rot, contact_normal);
                if (clamped.dot(tri.normal) > 0) {
                    contact_normal = clamped;
                    contact_point = pos_on_a - contact_normal * distance;
                }
            }
            return;
        }

        if (contact_normal.dot(tri.normal) >= 0) {
            contact_normal = tri.normal;
            contact_point = pos_on_a - contact_normal * distance;
        }
    }

    struct RaycastCallback {
        TriangleMesh* mesh;
        Vec3 from, to;
        float t_max;
        Vec3 hit_normal, hit_point;
        bool has_hit;

        RS_INLINE void process_ray_node(int tri_idx, float& current_t_max) {
            Vec3 normal, point;
            if (mesh->triangles[tri_idx].intersect_ray(from, to, current_t_max, normal, point)) {
                t_max = current_t_max;
                hit_normal = normal;
                hit_point = point;
                has_hit = true;
            }
        }
    };

    RS_INLINE bool raycast(const Vec3& from, const Vec3& to, Vec3& out_normal, Vec3& out_point) const {
        if (num_tris == 0) return false;

        Vec3 direction = to - from;
        float ray_length = direction.length();
        if (ray_length < SIMD_EPSILON) return false;

        RaycastCallback cb;
        cb.mesh = const_cast<TriangleMesh*>(this);
        cb.from = from;
        cb.to = to;
        cb.t_max = 1.0f;
        cb.has_hit = false;

        bvh.report_ray_overlapping_node(cb, from, direction, cb.t_max);

        if (cb.has_hit) {
            out_normal = cb.hit_normal.normalized();
            out_point = cb.hit_point;
        }
        return cb.has_hit;
    }
};

struct ArenaMeshCollection {
    TriangleMesh** meshes;
    int num_meshes;

    RS_INLINE ArenaMeshCollection() : meshes(nullptr), num_meshes(0) {}
    RS_INLINE ArenaMeshCollection(TriangleMesh** m, int n) : meshes(m), num_meshes(n) {}

    RS_INLINE bool raycast(const Vec3& from, const Vec3& to, Vec3& out_normal, Vec3& out_point) const {
        bool best_hit = false;
        float best_dist_sq = 1e18f;
        Vec3 best_normal, best_point;

        for (int i = 0; i < num_meshes; i++) {
            if (meshes[i] == nullptr || meshes[i]->num_tris == 0) continue;

            Vec3 normal, point;
            if (meshes[i]->raycast(from, to, normal, point)) {
                float dist_sq = (point - from).length_squared();
                if (dist_sq < best_dist_sq) {
                    best_dist_sq = dist_sq;
                    best_normal = normal;
                    best_point = point;
                    best_hit = true;
                }
            }
        }

        if (best_hit) {
            out_normal = best_normal;
            out_point = best_point;
        }
        return best_hit;
    }
};

struct CollisionWorld {
    RigidBody bodies[MAX_BODIES];
    PersistentManifold manifolds[MAX_MANIFOLDS];
    int non_static[MAX_BODIES];
    int num_bodies;
    int num_manifolds;
    int num_non_static;
    TriangleMesh* arena_meshes[NUM_ARENA_MESHES];
    int num_arena_meshes;
    ConstraintSolver solver;
    BallContactDebug ball_contact_debug;
    BallCarCollisionState ball_car_collision_state;

    RS_INLINE CollisionWorld() : num_bodies(0), num_manifolds(0), num_non_static(0), num_arena_meshes(0) {
        for (int i = 0; i < NUM_ARENA_MESHES; i++) arena_meshes[i] = nullptr;
    }

    RS_INLINE int add_body(const RigidBody& rb, bool is_static) {
        if (num_bodies >= MAX_BODIES) return -1;
        int idx = num_bodies++;
        bodies[idx] = rb;
        bodies[idx].collision.world_idx = idx;
        bodies[idx].collision.flags = is_static ? CF_STATIC : 0;
        if (!is_static) {
            bodies[idx].set_gravity(Vec3(0, 0, physics::to_phys(GRAVITY_Z)));
            non_static[num_non_static++] = idx;
        }
        return idx;
    }

    RS_INLINE void apply_gravity(float dt) {
        for (int i = 0; i < num_non_static; i++) {
            RigidBody& rb = bodies[non_static[i]];
            if (rb.collision.is_active()) {
                rb.apply_gravity();
            }
        }
    }

    RS_INLINE void predict_motion(float dt) {
        for (int i = 0; i < num_non_static; i++) {
            RigidBody& rb = bodies[non_static[i]];
            rb.apply_damping(dt);
            rb.collision.interp_transform = rb.predict_transform(dt);
        }
    }

    RS_INLINE void add_contact(int b0, int b1, const Vec3& pos_a, const Vec3& pos_b, const Vec3& normal, float dist, float fric, float rest, ManifoldType mtype, bool is_special = false) {
        if (is_special && b1 < 0) {
            Vec3 rel_pos = pos_a - bodies[b0].collision.transform.translation;
            solver.special.add(b0, normal, rel_pos.length(), rest, fric);
        }

        if (num_manifolds >= MAX_MANIFOLDS) return;

        int found = -1;
        for (int m = 0; m < num_manifolds; m++) {
            if ((manifolds[m].body0_idx == b0 && manifolds[m].body1_idx == b1) ||
                (manifolds[m].body0_idx == b1 && manifolds[m].body1_idx == b0)) {
                found = m;
                break;
            }
        }

        if (found < 0) {
            found = num_manifolds++;
            manifolds[found] = PersistentManifold();
            manifolds[found].body0_idx = b0;
            manifolds[found].body1_idx = b1;
            manifolds[found].type = mtype;

            float cbt0 = bodies[b0].collision.shape.get_contact_breaking_threshold(BASE_CONTACT_BREAKING_THRESHOLD);
            float cbt1 = (b1 >= 0) ? bodies[b1].collision.shape.get_contact_breaking_threshold(BASE_CONTACT_BREAKING_THRESHOLD)
                                    : cbt0;
            manifolds[found].breaking_threshold = fminf(cbt0, cbt1);
        }

        PersistentManifold& mf = manifolds[found];

        if (dist > mf.breaking_threshold) return;

        Vec3 local_a = bodies[b0].collision.transform.inv_xform(pos_a);
        Vec3 local_b = (b1 >= 0) ? bodies[b1].collision.transform.inv_xform(pos_b) : pos_b;

        ManifoldPoint cp(local_a, local_b, normal, dist);
        cp.world_a = pos_a;
        cp.world_b = pos_b;
        cp.friction = fric;
        cp.restitution = rest;
        cp.is_special = is_special;
        mf.add_manifold_point(cp);
    }

    RS_INLINE void detect_sphere_sphere(int idx_a, int idx_b) {
        RigidBody& a = bodies[idx_a];
        RigidBody& b = bodies[idx_b];
        if (a.collision.shape.type != SHAPE_SPHERE || b.collision.shape.type != SHAPE_SPHERE) return;

        Vec3 diff = b.collision.transform.translation - a.collision.transform.translation;
        float dist = diff.length();
        float r_sum = a.collision.shape.sphere.radius + b.collision.shape.sphere.radius;

        if (dist < r_sum) {
            Vec3 normal = (dist > SIMD_EPSILON) ? diff * (1.0f / dist) : Vec3(1, 0, 0);
            float pen = r_sum - dist;
            Vec3 pos_a = a.collision.transform.translation + normal * a.collision.shape.sphere.radius;
            Vec3 pos_b = b.collision.transform.translation - normal * b.collision.shape.sphere.radius;

            float fric = PersistentManifold::calc_friction(a.collision.is_static(), b.collision.is_static(), a.collision.friction, b.collision.friction);
            float rest = PersistentManifold::calc_restitution(a.collision.is_static(), b.collision.is_static(), a.collision.restitution, b.collision.restitution);
            add_contact(idx_a, idx_b, pos_a, pos_b, normal, -pen, fric, rest, MANIFOLD_SPHERE_SPHERE);
        }
    }

    RS_INLINE void detect_sphere_box(int sphere_idx, int box_idx) {
        RigidBody& sphere = bodies[sphere_idx];
        RigidBody& box_body = bodies[box_idx];
        if (sphere.collision.shape.type != SHAPE_SPHERE || box_body.collision.shape.type != SHAPE_BOX) return;

        Affine3 box_trans = box_body.collision.get_hitbox_transform();

        Vec3 normal, point;
        float depth;
        bool collided = sphere_obb_collision(
            sphere.collision.transform.translation, sphere.collision.shape.sphere.radius,
            box_trans, box_body.collision.shape.box.implicit_shape_dimensions,
            box_body.collision.shape.box.collision_margin,
            normal, point, depth
        );

        if (collided) {
            Vec3 pos_a = sphere.collision.transform.translation - normal * sphere.collision.shape.sphere.radius;

            float fric, rest;
            bool is_ball_car = (sphere.collision.user_type == USER_TYPE_BALL && box_body.collision.user_type == USER_TYPE_CAR) ||
                               (sphere.collision.user_type == USER_TYPE_CAR && box_body.collision.user_type == USER_TYPE_BALL);
            if (is_ball_car) {
                fric = car::HIT_BALL_FRICTION;
                rest = car::HIT_BALL_RESTITUTION;

                RigidBody* ball_rb;
                RigidBody* car_rb;
                if (sphere.collision.user_type == USER_TYPE_BALL) {
                    ball_rb = &sphere;
                    car_rb = &box_body;
                } else {
                    ball_rb = &box_body;
                    car_rb = &sphere;
                }
                ball_car_collision_state.ball_pos = ball_rb->collision.transform.translation * physics::INV_SCALE;
                ball_car_collision_state.ball_vel = ball_rb->linear_vel * physics::INV_SCALE;
                ball_car_collision_state.car_pos = car_rb->collision.transform.translation * physics::INV_SCALE;
                ball_car_collision_state.car_vel = car_rb->linear_vel * physics::INV_SCALE;
                ball_car_collision_state.car_forward = car_rb->collision.transform.matrix3.x_axis;
                ball_car_collision_state.car_user_id = car_rb->collision.user_id;
                ball_car_collision_state.has_collision = true;
            } else {
                fric = PersistentManifold::calc_friction(sphere.collision.is_static(), box_body.collision.is_static(), sphere.collision.friction, box_body.collision.friction);
                rest = PersistentManifold::calc_restitution(sphere.collision.is_static(), box_body.collision.is_static(), sphere.collision.restitution, box_body.collision.restitution);
            }
            add_contact(sphere_idx, box_idx, pos_a, point, normal, depth, fric, rest, MANIFOLD_SPHERE_BOX);
        }
    }

    struct SphereMeshCallback {
        CollisionWorld* world;
        TriangleMesh* mesh;
        int sphere_idx;
        int mesh_id;
        Vec3 center;
        float radius;
        float threshold;
        bool is_ball;

        RS_INLINE void process_node(int tri_idx) {
            TriangleShape& tri = mesh->triangles[tri_idx];
            Vec3 normal, point;
            float depth;
            if (tri.intersect_sphere(center, radius, threshold, normal, point, depth)) {
                Vec3 pos_a = center - normal * radius;
                mesh->adjust_contact_for_internal_edge(tri_idx, normal, point, pos_a, depth);
                float fric = fminf(world->bodies[sphere_idx].collision.friction, arena::FRICTION);
                float rest = fmaxf(world->bodies[sphere_idx].collision.restitution, arena::RESTITUTION);
                world->add_contact(sphere_idx, mesh_id, pos_a, point, normal, depth, fric, rest, MANIFOLD_SPHERE_MESH, is_ball);

                if (is_ball) {
                    BallContactDebug& debug = world->ball_contact_debug;
                    if (!debug.has_contact || depth < debug.depth) {
                        debug.normal = normal;
                        debug.depth = depth;
                        debug.tri_idx = tri_idx;
                        debug.has_contact = true;
                    }
                }
            }
        }
    };

    RS_INLINE void detect_sphere_mesh(int sphere_idx) {
        RigidBody& sphere = bodies[sphere_idx];
        if (sphere.collision.shape.type != SHAPE_SPHERE) return;
        if (num_arena_meshes == 0) return;

        float radius = sphere.collision.shape.sphere.radius;
        Vec3 center = sphere.collision.transform.translation;
        Aabb sphere_aabb = sphere.collision.shape.sphere.get_aabb(sphere.collision.transform);
        Aabb interp_aabb = sphere.collision.shape.sphere.get_aabb(sphere.collision.interp_transform);
        Aabb swept_aabb = sphere_aabb.merge(interp_aabb);
        bool is_ball = (sphere.collision.user_type == USER_TYPE_BALL);

        float threshold = radius * CONTACT_BREAKING_THRESHOLD;

        for (int m = 0; m < num_arena_meshes; m++) {
            TriangleMesh* mesh = arena_meshes[m];
            if (mesh == nullptr || mesh->num_tris == 0) continue;
            int mid = MESH_ID_BASE - m;
            SphereMeshCallback cb{ this, mesh, sphere_idx, mid, center, radius, threshold, is_ball };
            mesh->bvh.report_aabb_overlapping_node(cb, swept_aabb);
        }
    }

    struct BoxMeshCallback {
        CollisionWorld* world;
        TriangleMesh* mesh;
        int box_idx;
        int mesh_id;
        Affine3 box_trans;
        Vec3 half_ext;
        float box_threshold;
        float box_margin;

        RS_INLINE void process_node(int tri_idx) {
            TriangleShape& tri = mesh->triangles[tri_idx];
            Vec3 normal, point;
            float depth;
            if (box_triangle_collision(
                box_trans,
                half_ext,
                tri.verts,
                tri.normal,
                box_threshold,
                normal,
                point,
                depth
            )) {
                depth -= box_margin;
                Vec3 pos_a = point - normal * depth;
                mesh->adjust_contact_for_internal_edge(tri_idx, normal, point, pos_a, depth);
                world->add_contact(box_idx, mesh_id, pos_a, point, normal, depth, car::HIT_WORLD_FRICTION, car::HIT_WORLD_RESTITUTION, MANIFOLD_BOX_MESH);
            }
        }
    };

    RS_INLINE void detect_box_mesh(int box_idx) {
        RigidBody& box_body = bodies[box_idx];
        if (box_body.collision.shape.type != SHAPE_BOX) return;
        if (num_arena_meshes == 0) return;

        Affine3 box_trans = box_body.collision.get_hitbox_transform();
        Vec3 half_ext = box_body.collision.shape.box.get_half_extents();
        Aabb box_aabb = box_body.collision.shape.box.get_aabb(box_trans);
        Affine3 interp_hitbox = box_body.collision.interp_transform;
        interp_hitbox.translation = interp_hitbox.translation + interp_hitbox.matrix3 * box_body.collision.hitbox_offset;
        Aabb interp_aabb = box_body.collision.shape.box.get_aabb(interp_hitbox);
        Aabb swept_aabb = box_aabb.merge(interp_aabb);
        float box_threshold = CONVEX_DISTANCE_MARGIN;
        float box_margin = box_body.collision.shape.box.collision_margin;

        for (int m = 0; m < num_arena_meshes; m++) {
            TriangleMesh* mesh = arena_meshes[m];
            if (mesh == nullptr || mesh->num_tris == 0) continue;
            int mid = MESH_ID_BASE - m;
            BoxMeshCallback cb{ this, mesh, box_idx, mid, box_trans, half_ext, box_threshold, box_margin };
            mesh->bvh.report_aabb_overlapping_node(cb, swept_aabb);
        }
    }

    RS_INLINE void detect_sphere_plane(int sphere_idx, const Vec3& plane_pos, const Vec3& plane_normal, int plane_id) {
        RigidBody& sphere = bodies[sphere_idx];
        if (sphere.collision.shape.type != SHAPE_SPHERE) return;

        float radius = sphere.collision.shape.sphere.radius;
        Vec3 center = sphere.collision.transform.translation;
        float dist_to_plane = plane_normal.dot(center - plane_pos);

        float threshold = radius * CONTACT_BREAKING_THRESHOLD;
        if (dist_to_plane >= radius + threshold) return;

        float depth = dist_to_plane - radius;
        Vec3 contact_point = center - plane_normal * dist_to_plane;
        Vec3 pos_a = center - plane_normal * radius;

        bool is_ball = (sphere.collision.user_type == USER_TYPE_BALL);
        float fric = fminf(sphere.collision.friction, arena::FRICTION);
        float rest = fmaxf(sphere.collision.restitution, arena::RESTITUTION);
        add_contact(sphere_idx, plane_id, pos_a, contact_point, plane_normal, depth, fric, rest, MANIFOLD_PLANE, is_ball);
    }

    RS_INLINE void detect_box_plane(int box_idx, const Vec3& plane_pos, const Vec3& plane_normal, int plane_id) {
        RigidBody& box_body = bodies[box_idx];
        if (box_body.collision.shape.type != SHAPE_BOX) return;

        Affine3 box_trans = box_body.collision.get_hitbox_transform();
        Vec3 half_ext = box_body.collision.shape.box.get_half_extents();

        Vec3 local_normal = box_trans.matrix3.transpose() * (plane_normal * -1.0f);
        Vec3 support = Vec3(
            local_normal.x >= 0 ? half_ext.x : -half_ext.x,
            local_normal.y >= 0 ? half_ext.y : -half_ext.y,
            local_normal.z >= 0 ? half_ext.z : -half_ext.z
        );
        Vec3 vtx_world = box_trans.matrix3 * support + box_trans.translation;

        float dist = plane_normal.dot(vtx_world - plane_pos);
        float box_threshold = CONVEX_DISTANCE_MARGIN;
        if (dist >= box_threshold) return;

        Vec3 contact_point = vtx_world - plane_normal * dist;
        bool is_car = (box_body.collision.user_type == USER_TYPE_CAR);
        float fric = is_car ? car::HIT_WORLD_FRICTION : arena::FRICTION;
        float rest = is_car ? car::HIT_WORLD_RESTITUTION : arena::RESTITUTION;
        add_contact(box_idx, plane_id, vtx_world, contact_point, plane_normal, dist, fric, rest, MANIFOLD_PLANE);
    }

    RS_INLINE void detect_arena_planes(int body_idx) {
        Vec3 floor_pos = Vec3(0, 0, 0);
        Vec3 floor_normal = Vec3(0, 0, 1);
        Vec3 ceiling_pos = Vec3(0, 0, arena::HEIGHT * physics::SCALE);
        Vec3 ceiling_normal = Vec3(0, 0, -1);
        Vec3 wall_x_neg_pos = Vec3(-arena::EXTENT_X * physics::SCALE, 0, 0);
        Vec3 wall_x_neg_normal = Vec3(1, 0, 0);
        Vec3 wall_x_pos_pos = Vec3(arena::EXTENT_X * physics::SCALE, 0, 0);
        Vec3 wall_x_pos_normal = Vec3(-1, 0, 0);

        RigidBody& body = bodies[body_idx];
        if (body.collision.shape.type == SHAPE_SPHERE) {
            detect_sphere_plane(body_idx, floor_pos, floor_normal, PLANE_FLOOR);
            detect_sphere_plane(body_idx, ceiling_pos, ceiling_normal, PLANE_CEILING);
            detect_sphere_plane(body_idx, wall_x_neg_pos, wall_x_neg_normal, PLANE_WALL_X_NEG);
            detect_sphere_plane(body_idx, wall_x_pos_pos, wall_x_pos_normal, PLANE_WALL_X_POS);
        } else if (body.collision.shape.type == SHAPE_BOX) {
            detect_box_plane(body_idx, floor_pos, floor_normal, PLANE_FLOOR);
            detect_box_plane(body_idx, ceiling_pos, ceiling_normal, PLANE_CEILING);
            detect_box_plane(body_idx, wall_x_neg_pos, wall_x_neg_normal, PLANE_WALL_X_NEG);
            detect_box_plane(body_idx, wall_x_pos_pos, wall_x_pos_normal, PLANE_WALL_X_POS);
        }
    }

    RS_INLINE void refresh_persistent_manifolds() {
        for (int m = num_manifolds - 1; m >= 0; m--) {
            PersistentManifold& mf = manifolds[m];

            if (!mf.should_persist()) {
                mf.clear();
                continue;
            }

            if (mf.num_points == 0) {
                if (m != num_manifolds - 1) manifolds[m] = manifolds[num_manifolds - 1];
                num_manifolds--;
                continue;
            }

            int b0 = mf.body0_idx;
            int b1 = mf.body1_idx;
            if (b0 < 0) {
                if (m != num_manifolds - 1) manifolds[m] = manifolds[num_manifolds - 1];
                num_manifolds--;
                continue;
            }

            Affine3 trans_a = bodies[b0].collision.transform;
            Affine3 trans_b = (b1 >= 0) ? bodies[b1].collision.transform : Affine3::identity();
            mf.refresh_contact_points(trans_a, trans_b);

            if (mf.num_points == 0) {
                if (m != num_manifolds - 1) manifolds[m] = manifolds[num_manifolds - 1];
                num_manifolds--;
            }
        }
    }

    RS_INLINE void perform_collision_detection() {
        refresh_persistent_manifolds();
        solver.special.reset();
        ball_contact_debug.reset();
        ball_car_collision_state.reset();

        for (int i = 0; i < num_non_static; i++) {
            int idx = non_static[i];
            detect_arena_planes(idx);
            detect_sphere_mesh(idx);
            detect_box_mesh(idx);

            for (int j = i + 1; j < num_non_static; j++) {
                int jdx = non_static[j];
                detect_sphere_sphere(idx, jdx);
                detect_sphere_box(idx, jdx);
                detect_sphere_box(jdx, idx);
                detect_box_box(idx, jdx);
            }
        }
    }

    RS_INLINE void detect_box_box(int idx_a, int idx_b) {
        RigidBody& a = bodies[idx_a];
        RigidBody& b = bodies[idx_b];
        if (a.collision.shape.type != SHAPE_BOX || b.collision.shape.type != SHAPE_BOX) return;

        Affine3 trans_a = a.collision.get_hitbox_transform();
        Affine3 trans_b = b.collision.get_hitbox_transform();

        Mat3 xforma = trans_a.matrix3.transpose();
        Mat3 xformb = trans_b.matrix3.transpose();

        Obb obb1(trans_a.translation, xforma, a.collision.shape.box.get_half_extents());
        Obb obb2(trans_b.translation, xformb, b.collision.shape.box.get_half_extents());

        SatHit hit = box_box_sat(obb1, trans_a.matrix3, obb2);
        if (!hit.valid) return;

        ManifoldPoint points[MANIFOLD_CACHE_SIZE];
        int num_points = 0;
        compute_box_box_contacts(obb1, trans_a.matrix3, obb2, trans_b.matrix3, hit, points, num_points);

        float fric = car::HIT_CAR_FRICTION;
        float rest = car::HIT_CAR_RESTITUTION;

        for (int i = 0; i < num_points; i++) {
            add_contact(idx_a, idx_b, points[i].local_a, points[i].local_b, points[i].normal, points[i].distance, fric, rest, MANIFOLD_BOX_BOX);
        }
    }

    RS_INLINE void solve_constraints(float dt) {
        solver.solve(bodies, num_bodies, manifolds, num_manifolds, dt);
    }

    RS_INLINE void integrate_transforms(float dt) {
        for (int i = 0; i < num_non_static; i++) {
            RigidBody& rb = bodies[non_static[i]];
            if (!rb.collision.is_active()) continue;
            Affine3 pred = rb.predict_transform(dt);
            rb.set_centre_of_mass_trans(pred);
        }
    }

    RS_INLINE void clear_forces() {
        for (int i = 0; i < num_non_static; i++) {
            bodies[non_static[i]].clear_forces();
        }
    }

    RS_INLINE void update_activation_state(float dt) {
        for (int i = 0; i < num_non_static; i++) {
            bodies[non_static[i]].update_activation_state(dt);
        }
    }

    RS_INLINE void step(float dt) {
        apply_gravity(dt);
        predict_motion(dt);
        perform_collision_detection();
        solve_constraints(dt);
        integrate_transforms(dt);
        update_activation_state(dt);
        clear_forces();
    }
};

struct Arena {
    CollisionWorld world;
    Car cars[2];
    Ball ball;
    BoostPad pads[TOTAL_PADS];
    int num_cars;
    int ball_body_idx;
    int ground_body_idx;

    RS_INLINE Arena() : num_cars(0), tick_count(0), ball_body_idx(-1), ground_body_idx(-1) {
        for (int i = 0; i < NUM_BIG_PADS; i++) pads[i].init(get_big_pad_loc(i), true);
        for (int i = 0; i < NUM_SMALL_PADS; i++) pads[NUM_BIG_PADS + i].init(get_small_pad_loc(i), false);

        RigidBody ground_body;
        ground_body.init(0.0f, Vec3::zero(), Affine3::identity(), CollisionShape(), 0.0f, 0.0f);
        ground_body_idx = world.add_body(ground_body, true);
    }

    RS_INLINE void init_ball() {
        ball.init_body();
        ball_body_idx = world.add_body(ball.body, false);
        world.bodies[ball_body_idx].collision.user_type = USER_TYPE_BALL;
        world.bodies[ball_body_idx].collision.user_id = 0;
        world.bodies[ball_body_idx].can_sleep = true;
    }

    RS_INLINE int add_car(int team) {
        if (num_cars >= 8) return -1;
        Car& car = cars[num_cars];
        car.team = team;
        car.id = num_cars;
        car.config = make_car_config(CAR_TYPE_OCTANE);
        car.init_vehicle();

        float mass = car::MASS;
        Vec3 half_ext_raw = car.config.hitbox_size * 0.5f * physics::SCALE;
        BoxShape box(half_ext_raw);
        Vec3 half_ext = box.get_half_extents();
        Vec3 inertia = box.calculate_local_inertia(mass);

        CollisionShape shape;
        shape.type = SHAPE_BOX;
        shape.box = box;

        Affine3 trans;
        trans.translation = car.state.pos * physics::SCALE;
        trans.matrix3 = car.state.rot;

        car.body.init(mass, inertia, trans, shape, car::FRICTION, car::RESTITUTION);

        car.body.collision.hitbox_offset = car.config.hitbox_offset * physics::SCALE;
        int idx = world.add_body(car.body, false);
        car.body_idx = idx;
        world.bodies[idx].collision.user_type = USER_TYPE_CAR;
        world.bodies[idx].collision.user_id = num_cars;
        return num_cars++;
    }

    RS_INLINE void update_boost_pads(float dt) {
        for (int p = 0; p < TOTAL_PADS; p++) {
            pads[p].update(dt);
        }
    }

    unsigned long long tick_count;

    RS_INLINE void process_ball_car_collision(int car_idx, const Vec3& local_point_on_ball) {
        Car& car = cars[car_idx];
        if (car.state.is_demoed) return;

        BallHitInfo ball_hit_info;
        ball_hit_info.is_valid = true;
        ball_hit_info.relative_pos = local_point_on_ball;
        ball_hit_info.tick_count_when_hit = tick_count;
        ball_hit_info.ball_pos = ball.state.pos;
        ball_hit_info.extra_hit_vel = Vec3::zero();
        ball_hit_info.tick_count_when_extra_impulse_applied = tick_count;

        Vec3 car_forward = car.state.rot.x_axis;
        Vec3 rel_pos = ball.state.pos - car.state.pos;
        Vec3 rel_vel = ball.state.vel - car.state.vel;

        float rel_speed = fminf(rel_vel.length(), ball::car_hit::MAX_DELTA_VEL);

        bool can_accel = ball.last_extra_hit_tick < (long long)tick_count - 1;

        if (rel_speed > 0.0f && can_accel) {
            float z_scale = ball::car_hit::Z_SCALE_NORMAL;
            Vec3 hit_dir = rel_pos * Vec3(1.0f, 1.0f, z_scale).normalized();
            float fwd_dot = hit_dir.dot(car_forward);
            Vec3 fwd_adj = car_forward * fwd_dot * (1.0f - ball::car_hit::FORWARD_SCALE);
            hit_dir = (hit_dir - fwd_adj).normalized();

            float impulse_scale = curves::eval_ball_car_extra_impulse(rel_speed);
            Vec3 extra_vel = hit_dir * rel_speed * impulse_scale;

            ball_hit_info.extra_hit_vel = extra_vel;
            ball.velocity_impulse_cache = ball.velocity_impulse_cache + extra_vel * physics::SCALE;
            ball.last_extra_hit_tick = (long long)tick_count;
        }

        car.state.has_ball_hit = true;
        car.state.ball_hit_info = ball_hit_info;
    }

    RS_INLINE void process_car_car_collision(int car1_idx, int car2_idx, const Vec3& local_a, const Vec3& local_b, float dt) {
        Car& car1 = cars[car1_idx];
        Car& car2 = cars[car2_idx];

        if (car1.state.is_demoed || car2.state.is_demoed) return;

        for (int swap = 0; swap < 2; swap++) {
            Car& attacker = (swap == 0) ? car1 : car2;
            Car& victim = (swap == 0) ? car2 : car1;

            if (attacker.state.bump_cooldown_timer > 0.0f) continue;

            Vec3 delta_pos = victim.state.pos - attacker.state.pos;
            if (attacker.state.vel.dot(delta_pos) < 0.0f) continue;

            Vec3 vel_dir = attacker.state.vel.normalized();
            Vec3 dir_to_victim = delta_pos.normalized();

            float speed_towards = attacker.state.vel.dot(dir_to_victim);
            float victim_away_speed = victim.state.vel.dot(vel_dir);
            if (speed_towards <= victim_away_speed) continue;

            bool is_demo = attacker.state.is_supersonic;
            if (is_demo && attacker.team == victim.team) is_demo = false;

            if (is_demo) {
                victim.state.is_demoed = true;
                victim.state.demo_respawn_timer = car::spawn::RESPAWN_TIME;
            } else {
                bool ground_hit = victim.state.is_on_ground;
                float base_scale = ground_hit ?
                    curves::eval_bump_vel_ground(speed_towards) :
                    curves::eval_bump_vel_air(speed_towards);

                Vec3 hit_up = victim.state.is_on_ground ? victim.state.rot.z_axis : Vec3(0, 0, 1);
                Vec3 bump_vel = vel_dir * base_scale + hit_up * curves::eval_bump_up_vel(speed_towards);

                victim.velocity_impulse_cache = victim.velocity_impulse_cache + bump_vel * physics::SCALE;
            }

            attacker.state.bump_cooldown_timer = car::bump::COOLDOWN_TIME;

        }
    }

    RS_INLINE void process_collision_callbacks(float dt) {
        for (int m = 0; m < world.num_manifolds; m++) {
            PersistentManifold& mf = world.manifolds[m];
            if (mf.num_points == 0) continue;

            int b0 = mf.body0_idx;
            int b1 = mf.body1_idx;

            if (b0 < 0) continue;

            if (b1 < 0) {
                RigidBody& rb0 = world.bodies[b0];
                if (rb0.collision.user_type == USER_TYPE_CAR) {
                    cars[rb0.collision.user_id].state.has_world_contact = true;
                    cars[rb0.collision.user_id].state.world_contact_normal = mf.points[0].normal;
                }
                continue;
            }

            RigidBody& rb0 = world.bodies[b0];
            RigidBody& rb1 = world.bodies[b1];

            int type0 = rb0.collision.user_type;
            int type1 = rb1.collision.user_type;

            if (type0 == USER_TYPE_CAR && type1 == USER_TYPE_CAR) {
                process_car_car_collision(rb0.collision.user_id, rb1.collision.user_id, mf.points[0].local_a, mf.points[0].local_b, dt);
            }
            else if ((type0 == USER_TYPE_BALL && type1 == USER_TYPE_CAR) ||
                     (type0 == USER_TYPE_CAR && type1 == USER_TYPE_BALL)) {
                int car_id = (type0 == USER_TYPE_CAR) ? rb0.collision.user_id : rb1.collision.user_id;
                Vec3 local_point = (type0 == USER_TYPE_BALL) ? mf.points[0].local_a : mf.points[0].local_b;
                process_ball_car_collision(car_id, local_point);
            }
            else if (type0 == USER_TYPE_CAR && type1 == USER_TYPE_NONE) {
                cars[rb0.collision.user_id].state.has_world_contact = true;
                cars[rb0.collision.user_id].state.world_contact_normal = mf.points[0].normal;
            }
            else if (type0 == USER_TYPE_NONE && type1 == USER_TYPE_CAR) {
                cars[rb1.collision.user_id].state.has_world_contact = true;
                cars[rb1.collision.user_id].state.world_contact_normal = mf.points[0].normal * -1.0f;
            }
        }
    }

    RS_INLINE void step(float dt) {
        RigidBody& ball_rb = world.bodies[ball_body_idx];
        bool should_sleep = ball_rb.linear_vel.length_squared() == 0.0f
            && ball_rb.angular_vel.length_squared() == 0.0f;
        ball_rb.collision.state = should_sleep ? SLEEPING : ACTIVE;

        for (int c = 0; c < num_cars; c++) {
            Car& car = cars[c];
            RigidBody& car_rb = world.bodies[car.body_idx];

            if (car.state.is_demoed) {
                car.state.demo_respawn_timer = fmaxf(0.0f, car.state.demo_respawn_timer - dt);
                if (car.state.demo_respawn_timer == 0.0f) {
                    int spawn_idx = (int)(tick_count % car::spawn::NUM_RESPAWN_LOCS);
                    float yaw = car::spawn::RESPAWN_YAW + (car.team == 0 ? 0.0f : 3.14159265f);
                    float cy = cosf(yaw), sy = sinf(yaw);

                    CarState fresh;
                    fresh.pos = car::spawn::get_respawn_pos(spawn_idx, car.team);
                    fresh.rot = Mat3(Vec3(cy, sy, 0), Vec3(-sy, cy, 0), Vec3(0, 0, 1));
                    fresh.vel = Vec3::zero();
                    fresh.ang_vel = Vec3::zero();
                    fresh.boost_amount = car::boost::SPAWN_AMOUNT;
                    car.state = fresh;
                    car_rb.collision.transform.translation = physics::to_phys(car.state.pos);
                    car_rb.collision.transform.matrix3 = car.state.rot;
                    car_rb.linear_vel = Vec3::zero();
                    car_rb.angular_vel = Vec3::zero();
                    car_rb.update_inertia();
                    car.velocity_impulse_cache = Vec3::zero();
                }
                car_rb.collision.state = DISABLED;
                car_rb.collision.flags |= CF_NO_RESPONSE;
                continue;
            }

            car_rb.collision.state = ACTIVE;
            car_rb.collision.flags &= ~CF_NO_RESPONSE;

            car.state.controls = car.state.controls.clamp();
            bool jump_pressed = car.state.controls.jump && !car.state.prev_controls.jump;

            int body_idx = car.body_idx;

            ArenaMeshCollection arena_collection(world.arena_meshes, world.num_arena_meshes);
            car.update_vehicle_first(dt, world.bodies, ground_body_idx, &arena_collection);
            car.update_wheel_contacts();
            float fwd_speed = world.bodies[body_idx].get_forward_speed() * physics::INV_SCALE;
            car.update_wheels(dt, fwd_speed, world.bodies);

            if (!car.state.is_on_ground) {
                car.update_air_torque(car.count_wheel_contacts() == 0, world.bodies);
            } else {
                car.state.is_flipping = false;
            }

            car.update_jump(dt, jump_pressed, world.bodies);
            car.update_auto_flip(dt, jump_pressed, world.bodies);
            car.update_flip(dt, jump_pressed, fwd_speed, world.bodies);

            int num_wheel_contact = car.count_wheel_contacts();
            if (car.state.controls.throttle != 0 &&
                ((num_wheel_contact > 0 && num_wheel_contact < 4) || car.state.has_world_contact)) {
                car.update_auto_roll(num_wheel_contact, world.bodies);
            }
            car.state.has_world_contact = false;
            car.state.has_ball_hit = false;
            car.state.has_car_contact = false;

            car.update_vehicle_second(dt, world.bodies);
            car.update_boost(dt, world.bodies);
        }

        world.step(dt);

        process_collision_callbacks(dt);

        for (int c = 0; c < num_cars; c++) {
            Car& car = cars[c];
            if (car.state.is_demoed) continue;
            int body_idx = car.body_idx;
            car.update_from_body(world.bodies[body_idx]);
            car.update_supersonic(dt);
            car.state.bump_cooldown_timer = fmaxf(0.0f, car.state.bump_cooldown_timer - dt);
            car.state.prev_controls = car.state.controls;
            car.finish_physics_tick(world.bodies[body_idx]);
            for (int p = 0; p < TOTAL_PADS; p++) {
                pads[p].try_pickup(car.state.pos, car.state.boost_amount);
            }
        }

        ball.finish_physics_tick(world.bodies[ball_body_idx]);

        update_boost_pads(dt);
        tick_count++;
    }
};

struct CollisionMeshFile {
    uint32_t* indices;
    Vec3* vertices;
    uint32_t num_tris;
    uint32_t num_verts;
    uint32_t hash;

    CollisionMeshFile() : indices(nullptr), vertices(nullptr), num_tris(0), num_verts(0), hash(0) {}

    ~CollisionMeshFile() {
        if (indices) delete[] indices;
        if (vertices) delete[] vertices;
    }

    uint32_t calculate_hash() {
        const uint32_t HASH_VAL_MUELLER = 0x45D9F3B;
        const uint32_t HASH_VAL_SHIFT = 0x9E3779B9;

        uint32_t h = (uint32_t)(num_verts + (num_tris * num_verts));

        uint32_t num_indices = num_tris * 3;
        for (uint32_t i = 0; i < num_indices; i++) {
            uint32_t vert_index = indices[i];
            Vec3& v = vertices[vert_index];
            float pos[3] = { v.x, v.y, v.z };

            for (int j = 0; j < 3; j++) {
                uint32_t cur_val = (uint32_t)(int32_t)pos[j];
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = (cur_val >> 16) ^ cur_val;
                h ^= cur_val + HASH_VAL_SHIFT + (h << 6) + (h >> 2);
            }
        }

        return h;
    }

    bool read_from_file(const char* path) {
        FILE* file = fopen(path, "rb");
        if (!file) return false;

        fread(&num_tris, sizeof(uint32_t), 1, file);
        fread(&num_verts, sizeof(uint32_t), 1, file);

        if (num_tris == 0 || num_verts == 0 || num_tris > 1000000 || num_verts > 1000000) {
            fprintf(stderr, "Invalid mesh file (bad triangle/vertex count: [%u/%u])\n", num_tris, num_verts);
            fclose(file);
            return false;
        }

        uint32_t num_indices = num_tris * 3;
        indices = new uint32_t[num_indices];
        fread(indices, sizeof(uint32_t), num_indices, file);

        vertices = new Vec3[num_verts];
        for (uint32_t i = 0; i < num_verts; i++) {
            float x, y, z;
            fread(&x, sizeof(float), 1, file);
            fread(&y, sizeof(float), 1, file);
            fread(&z, sizeof(float), 1, file);
            vertices[i] = Vec3(x, y, z);
        }

        fclose(file);

        hash = calculate_hash();
        return true;
    }

    void add_to_triangle_mesh(TriangleMesh& mesh) {
        for (uint32_t i = 0; i < num_tris; i++) {
            uint32_t i0 = indices[i * 3 + 0];
            uint32_t i1 = indices[i * 3 + 1];
            uint32_t i2 = indices[i * 3 + 2];

            Vec3 p0 = vertices[i0];
            Vec3 p1 = vertices[i1];
            Vec3 p2 = vertices[i2];

            mesh.add_triangle(p0, p1, p2);
        }
    }
};

inline int load_soccar_arena(TriangleMesh** meshes, const char* mesh_folder) {
    char mesh_path[512];
    int num_loaded = 0;

    static const int MESH_ORDER[] = {0, 1, 10, 11, 12, 13, 14, 15, 2, 3, 4, 5, 6, 7, 8, 9};

    for (int idx = 0; idx < 16; idx++) {
        int mesh_num = MESH_ORDER[idx];
        snprintf(mesh_path, sizeof(mesh_path), "%s/soccar/mesh_%d.cmf", mesh_folder, mesh_num);

        CollisionMeshFile cmf;
        if (!cmf.read_from_file(mesh_path)) continue;

        TriangleMesh* mesh = new TriangleMesh();
        cmf.add_to_triangle_mesh(*mesh);

        if (mesh->num_tris > 0) {
            mesh->build_bvh();
            mesh->generate_edge_info();
            meshes[num_loaded++] = mesh;
        } else {
            delete mesh;
        }
    }

    return num_loaded;
}
