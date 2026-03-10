#pragma once

#include "body.cuh"
#include "../collision/manifold.cuh"
#include "../sim/constants.cuh"

constexpr float CONTACT_SOR = 1.0f;
constexpr float CONTACT_ERP = 0.2f;
constexpr float CONTACT_ERP2 = 0.8f;
constexpr float CONTACT_RESTITUTION_THRESH = 0.2f;
constexpr float CONTACT_SPLIT_PENETRATION_THRESH = 1e30f;
constexpr float CONTACT_SPLIT_TURN_ERP = 0.1f;
constexpr int CONTACT_NUM_ITERATIONS = 10;

RS_INLINE float bullet_dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + (a.y * b.y + a.z * b.z);
}

struct SolverBody {
    Affine3 world_transform;
    Vec3 delta_lin;
    Vec3 delta_ang;
    Vec3 push_vel;
    Vec3 turn_vel;
    Vec3 lin_vel;
    Vec3 ang_vel;
    Vec3 ext_force;
    Vec3 ext_torque;
    Vec3 inv_mass;
    int original_idx;

    RS_INLINE SolverBody() : inv_mass(Vec3::zero()), original_idx(-1) {
        delta_lin = delta_ang = push_vel = turn_vel = lin_vel = ang_vel = ext_force = ext_torque = Vec3::zero();
    }

    RS_INLINE void init_from_body(const RigidBody& rb, float dt) {
        world_transform = rb.collision.transform;
        inv_mass = Vec3::splat(rb.inv_mass);
        original_idx = rb.collision.world_idx;
        lin_vel = rb.linear_vel;
        ang_vel = rb.angular_vel;
        delta_lin = delta_ang = push_vel = turn_vel = Vec3::zero();

        ext_force = rb.total_force * rb.inv_mass * dt;
        ext_torque = rb.inv_inertia_world * rb.total_torque * dt;
    }

    RS_INLINE void apply_impulse(const Vec3& lin_comp, const Vec3& ang_comp, float impulse) {
        delta_lin = delta_lin + lin_comp * impulse;
        delta_ang = delta_ang + ang_comp * impulse;
    }

    RS_INLINE Vec3 get_vel_at_no_delta(const Vec3& rel) const {
        return lin_vel + ext_force + (ang_vel + ext_torque).cross(rel);
    }
};

struct FullSolverConstraint {
    Vec3 rel1_cross;
    Vec3 normal1;
    Vec3 rel2_cross;
    Vec3 normal2;
    Vec3 ang_comp_a;
    Vec3 ang_comp_b;
    float applied_push;
    float applied;
    float friction;
    float jac_inv;
    float rhs;
    float lower;
    float upper;
    float rhs_pen;
    int friction_idx;
    int body_a;
    int body_b;
    bool is_special;

    RS_INLINE FullSolverConstraint() : applied_push(0), applied(0), friction(0), jac_inv(0), rhs(0),
        lower(0), upper(1e10f), rhs_pen(0), friction_idx(0), body_a(0), body_b(0),
        is_special(false) {
        rel1_cross = normal1 = rel2_cross = normal2 = ang_comp_a = ang_comp_b = Vec3::zero();
    }

    RS_INLINE static float restitution_curve(float rel_vel, float rest) {
        return (fabsf(rel_vel) < CONTACT_RESTITUTION_THRESH) ? 0.0f : rest * -rel_vel;
    }

    RS_INLINE void setup_contact(
        const RigidBody& rb0, const RigidBody& rb1,
        SolverBody& sb0, SolverBody& sb1,
        const Vec3& rel1, const Vec3& rel2,
        const ManifoldPoint& cp, float dt
    ) {
        float inv_dt = (fabsf(dt) < 1e-12f) ? 0.0f : 1.0f / dt;
        applied = 0;

        Vec3 torque0 = rel1.cross(cp.normal);
        ang_comp_a = rb0.inv_inertia_world * torque0;
        Vec3 vec0 = ang_comp_a.cross(rel1);
        float denom0 = rb0.inv_mass + cp.normal.dot(vec0);
        Vec3 vel0 = rb0.get_velocity_at(rel1);
        normal1 = cp.normal;
        rel1_cross = torque0;
        sb0.apply_impulse(cp.normal * sb0.inv_mass, ang_comp_a, applied);

        float vel1_dot = bullet_dot(normal1, sb0.lin_vel + sb0.ext_force) + bullet_dot(rel1_cross, sb0.ang_vel + sb0.ext_torque);

        Vec3 torque1 = rel2.cross(cp.normal);
        ang_comp_b = rb1.inv_inertia_world * (torque1 * -1.0f);
        Vec3 vec1 = (ang_comp_b * -1.0f).cross(rel2);
        float denom1 = rb1.inv_mass + cp.normal.dot(vec1);
        Vec3 vel1 = rb1.get_velocity_at(rel2);
        normal2 = cp.normal * -1.0f;
        rel2_cross = torque1 * -1.0f;
        sb1.apply_impulse(cp.normal * sb1.inv_mass, ang_comp_b, -applied);

        float vel2_dot = bullet_dot(normal2, sb1.lin_vel + sb1.ext_force) + bullet_dot(rel2_cross, sb1.ang_vel + sb1.ext_torque);

        float denom_sum = denom0 + denom1;
        jac_inv = (fabsf(denom_sum) < 1e-12f) ? 0.0f : CONTACT_SOR / denom_sum;

        Vec3 vel = vel0 - vel1;
        float rel_vel = cp.normal.dot(vel);
        float rest = fmaxf(0.0f, restitution_curve(rel_vel, cp.restitution));

        float pen = cp.distance;
        float pos_err = (pen > 0) ? 0.0f : -pen * CONTACT_ERP2 * inv_dt;
        float rel_vel_total = vel1_dot + vel2_dot;
        float vel_err = rest - rel_vel_total;

        float pen_impulse = pos_err * jac_inv;
        float vel_impulse = vel_err * jac_inv;

        if (pen > CONTACT_SPLIT_PENETRATION_THRESH) {
            rhs = pen_impulse + vel_impulse;
            rhs_pen = 0;
        } else {
            rhs = vel_impulse;
            rhs_pen = pen_impulse;
        }
    }

    RS_INLINE void setup_friction(
        const RigidBody& rb0, const RigidBody& rb1,
        const SolverBody& sb0, const SolverBody& sb1,
        const Vec3& rel1, const Vec3& rel2,
        const Vec3& lat_dir
    ) {
        normal1 = lat_dir;
        rel1_cross = rel1.cross(normal1);
        ang_comp_a = rb0.inv_inertia_world * rel1_cross;
        Vec3 vec0 = ang_comp_a.cross(rel1);
        float denom0 = rb0.inv_mass + lat_dir.dot(vec0);
        float vel1_dot = bullet_dot(normal1, sb0.lin_vel + sb0.ext_force) + bullet_dot(rel1_cross, sb0.ang_vel);

        normal2 = lat_dir * -1.0f;
        rel2_cross = rel2.cross(normal2);
        ang_comp_b = rb1.inv_inertia_world * rel2_cross;
        Vec3 vec1 = (ang_comp_b * -1.0f).cross(rel2);
        float denom1 = rb1.inv_mass + lat_dir.dot(vec1);
        float vel2_dot = bullet_dot(normal2, sb1.lin_vel + sb1.ext_force) + bullet_dot(rel2_cross, sb1.ang_vel);

        float f_denom_sum = denom0 + denom1;
        jac_inv = (fabsf(f_denom_sum) < 1e-12f) ? 0.0f : CONTACT_SOR / f_denom_sum;
        float rel_vel = vel1_dot + vel2_dot;
        rhs = -rel_vel * jac_inv;
    }

    RS_INLINE float resolve_lower(SolverBody& a, SolverBody& b) {
        float delta = rhs;
        delta -= bullet_dot(normal1, a.delta_lin) * jac_inv;
        delta -= bullet_dot(rel1_cross, a.delta_ang) * jac_inv;
        delta -= bullet_dot(normal2, b.delta_lin) * jac_inv;
        delta -= bullet_dot(rel2_cross, b.delta_ang) * jac_inv;

        float sum = applied + delta;
        if (sum < lower) { delta = lower - applied; applied = lower; }
        else { applied = sum; }

        a.delta_lin = a.delta_lin + normal1 * a.inv_mass * delta;
        a.delta_ang = a.delta_ang + ang_comp_a * delta;
        b.delta_lin = b.delta_lin + normal2 * b.inv_mass * delta;
        b.delta_ang = b.delta_ang + ang_comp_b * delta;

        return delta / jac_inv;
    }

    RS_INLINE float resolve_generic(SolverBody& a, SolverBody& b) {
        float delta = rhs;
        delta -= bullet_dot(normal1, a.delta_lin) * jac_inv;
        delta -= bullet_dot(rel1_cross, a.delta_ang) * jac_inv;
        delta -= bullet_dot(normal2, b.delta_lin) * jac_inv;
        delta -= bullet_dot(rel2_cross, b.delta_ang) * jac_inv;

        float sum = applied + delta;
        if (sum < lower) { delta = lower - applied; applied = lower; }
        else if (sum > upper) { delta = upper - applied; applied = upper; }
        else { applied = sum; }

        a.delta_lin = a.delta_lin + normal1 * a.inv_mass * delta;
        a.delta_ang = a.delta_ang + ang_comp_a * delta;
        b.delta_lin = b.delta_lin + normal2 * b.inv_mass * delta;
        b.delta_ang = b.delta_ang + ang_comp_b * delta;

        return delta / jac_inv;
    }

    RS_INLINE float resolve_split(SolverBody& a, SolverBody& b) {
        if (rhs_pen == 0) return 0;

        float delta = rhs_pen;
        delta -= bullet_dot(normal1, a.push_vel) * jac_inv;
        delta -= bullet_dot(rel1_cross, a.turn_vel) * jac_inv;
        delta -= bullet_dot(normal2, b.push_vel) * jac_inv;
        delta -= bullet_dot(rel2_cross, b.turn_vel) * jac_inv;

        float sum = applied_push + delta;
        if (sum < lower) { delta = lower - applied_push; applied_push = lower; }
        else { applied_push = sum; }

        a.push_vel = a.push_vel + normal1 * a.inv_mass * delta;
        a.turn_vel = a.turn_vel + ang_comp_a * delta;
        b.push_vel = b.push_vel + normal2 * b.inv_mass * delta;
        b.turn_vel = b.turn_vel + ang_comp_b * delta;

        return delta / jac_inv;
    }
};

constexpr int MAX_SOLVER_BODIES = 16;
constexpr int MAX_CONSTRAINTS = 128;

struct SpecialResolveInfo {
    int object_index;
    int num_collisions;
    Vec3 total_normal;
    float total_dist;
    float restitution;
    float friction;

    RS_INLINE SpecialResolveInfo() : object_index(0), num_collisions(0), total_dist(0), restitution(0), friction(0) {
        total_normal = Vec3::zero();
    }

    RS_INLINE void add(int obj_idx, const Vec3& normal, float dist, float rest, float fric) {
        object_index = obj_idx;
        total_normal = total_normal + normal;
        total_dist += dist;
        restitution = rest;
        friction = fric;
        num_collisions++;
    }

    RS_INLINE void reset() {
        object_index = num_collisions = 0;
        total_dist = 0;
        restitution = friction = 0;
        total_normal = Vec3::zero();
    }
};

struct ConstraintSolver {
    SolverBody bodies[MAX_SOLVER_BODIES];
    FullSolverConstraint contacts[MAX_CONSTRAINTS];
    FullSolverConstraint friction[MAX_CONSTRAINTS];
    int num_bodies;
    int num_contacts;
    int num_friction;
    int fixed_body;
    SpecialResolveInfo special;

    RS_INLINE ConstraintSolver() : num_bodies(0), num_contacts(0), num_friction(0), fixed_body(-1) {}

    RS_INLINE int get_or_init_body(RigidBody& rb, float dt) {
        if (rb.collision.companion_id >= 0) return rb.collision.companion_id;

        if (!rb.collision.is_static() && rb.inv_mass != 0) {
            int id = num_bodies++;
            rb.collision.companion_id = id;
            bodies[id].init_from_body(rb, dt);
            return id;
        }

        if (fixed_body < 0) {
            fixed_body = num_bodies++;
            bodies[fixed_body] = SolverBody();
        }
        rb.collision.companion_id = fixed_body;
        return fixed_body;
    }

    RS_INLINE void convert_contact_special(const RigidBody& body, float dt) {
        if (special.num_collisions < 1) return;
        float dist = special.total_dist / special.num_collisions;
        Vec3 normal = special.total_normal * (1.0f / (float)special.num_collisions);

        int sb_a = body.collision.companion_id;
        int sb_b = fixed_body;
        if (sb_b < 0) {
            sb_b = num_bodies++;
            fixed_body = sb_b;
            bodies[sb_b] = SolverBody();
        }

        SolverBody& sba = bodies[sb_a];
        Vec3 rel = normal * -dist;
        float inv_dt = (fabsf(dt) < 1e-12f) ? 0.0f : 1.0f / dt;

        Vec3 torque = rel.cross(normal);
        Vec3 ang_comp = body.inv_inertia_world * torque;
        float denom = body.inv_mass + (ang_comp.cross(rel)).dot(normal);

        Vec3 vel = body.get_velocity_at(rel);
        float vel_rel = normal.dot(vel);
        float rest = fmaxf(0.0f, FullSolverConstraint::restitution_curve(vel_rel, special.restitution));
        float penetration = dist;
        float pen_err = (penetration > 0) ? 0 : -penetration * CONTACT_ERP2 * inv_dt;

        float jac_inv = (fabsf(denom) < 1e-12f) ? 0.0f : CONTACT_SOR / denom;

        float rel_vel = normal.dot(sba.lin_vel + sba.ext_force) + torque.dot(sba.ang_vel + sba.ext_torque);
        float vel_err = rest - rel_vel;
        float pen_impulse = pen_err * jac_inv;
        float vel_impulse = vel_err * jac_inv;

        if (num_contacts >= MAX_CONSTRAINTS || num_friction >= MAX_CONSTRAINTS) return;

        FullSolverConstraint& c = contacts[num_contacts++];
        c.body_a = sb_a;
        c.body_b = sb_b;
        c.is_special = false;
        c.normal1 = normal;
        c.rel1_cross = torque;
        c.ang_comp_a = ang_comp;
        c.jac_inv = jac_inv;
        c.lower = 0;
        c.upper = 1e10f;
        c.applied = 0;

        if (dist > CONTACT_SPLIT_PENETRATION_THRESH) {
            c.rhs = pen_impulse + vel_impulse;
            c.rhs_pen = 0;
        } else {
            c.rhs = vel_impulse;
            c.rhs_pen = pen_impulse;
        }

        c.normal2 = normal * -1.0f;
        c.rel2_cross = Vec3::zero();
        c.ang_comp_b = Vec3::zero();

        int contact_idx = num_contacts - 1;
        c.friction_idx = num_friction;

        Vec3 solver_vel = sba.get_vel_at_no_delta(rel);
        float solver_vel_rel = normal.dot(solver_vel);
        Vec3 lat_dir = solver_vel - normal * solver_vel_rel;
        float lat_len_sq = lat_dir.length_squared();
        lat_dir = (lat_len_sq > SIMD_EPSILON) ? lat_dir * rsqrtf(fmaxf(lat_len_sq, 1e-12f)) : plane_space_1(normal);

        FullSolverConstraint& f = friction[num_friction++];
        f.body_a = sb_a;
        f.body_b = sb_b;
        f.friction_idx = contact_idx;
        f.lower = -special.friction;
        f.upper = special.friction;
        f.friction = special.friction;
        f.applied = 0;

        f.normal1 = lat_dir;
        f.rel1_cross = rel.cross(lat_dir);
        f.ang_comp_a = body.inv_inertia_world * f.rel1_cross;
        float f_denom2 = body.inv_mass + (f.ang_comp_a.cross(rel)).dot(lat_dir);
        f.jac_inv = (fabsf(f_denom2) < 1e-12f) ? 0.0f : CONTACT_SOR / f_denom2;

        f.normal2 = lat_dir * -1.0f;
        f.rel2_cross = Vec3::zero();
        f.ang_comp_b = Vec3::zero();

        float f_vel_dot_n = lat_dir.dot(sba.lin_vel + sba.ext_force) + f.rel1_cross.dot(sba.ang_vel + sba.ext_torque);
        f.rhs = -f_vel_dot_n * f.jac_inv;
    }

    RS_INLINE int get_or_create_fixed_body() {
        if (fixed_body >= 0) return fixed_body;
        int id = num_bodies++;
        fixed_body = id;
        bodies[id] = SolverBody();
        bodies[id].original_idx = -1;
        return id;
    }

    RS_INLINE void setup(RigidBody* rbs, int num_rbs, PersistentManifold* manifolds, int num_manifolds, float dt) {
        num_bodies = num_contacts = num_friction = 0;
        fixed_body = -1;

        for (int i = 0; i < num_rbs; i++) rbs[i].collision.companion_id = -1;

        for (int i = 0; i < num_rbs; i++) {
            RigidBody& rb = rbs[i];
            if (!rb.collision.is_static() && rb.inv_mass != 0 && rb.collision.is_active()) {
                get_or_init_body(rb, dt);
            }
        }
        for (int m = 0; m < num_manifolds; m++) {
            PersistentManifold& mf = manifolds[m];

            bool is_static_collision = (mf.body1_idx < 0);

            RigidBody& b0 = rbs[mf.body0_idx];
            int sb0 = get_or_init_body(b0, dt);
            int sb1 = is_static_collision ? get_or_create_fixed_body() : get_or_init_body(rbs[mf.body1_idx], dt);

            for (int p = 0; p < mf.num_points; p++) {
                if (num_contacts >= MAX_CONSTRAINTS || num_friction >= MAX_CONSTRAINTS) break;

                ManifoldPoint& cp = mf.points[p];

                Vec3 rel1 = cp.world_a - b0.collision.transform.translation;
                Vec3 rel2 = is_static_collision ? Vec3::zero() : (cp.world_b - rbs[mf.body1_idx].collision.transform.translation);

                int contact_idx = num_contacts;
                FullSolverConstraint& contact = contacts[num_contacts++];
                contact.body_a = sb0;
                contact.body_b = sb1;
                contact.friction_idx = num_friction;
                contact.friction = cp.friction;
                contact.is_special = cp.is_special;
                contact.lower = 0;
                contact.upper = 1e10f;

                if (is_static_collision) {
                    float inv_dt = (fabsf(dt) < 1e-12f) ? 0.0f : 1.0f / dt;
                    contact.applied = 0;

                    Vec3 torque0 = rel1.cross(cp.normal);
                    contact.ang_comp_a = b0.inv_inertia_world * torque0;
                    Vec3 vec0 = contact.ang_comp_a.cross(rel1);
                    float denom0 = b0.inv_mass + cp.normal.dot(vec0);
                    Vec3 vel0 = b0.get_velocity_at(rel1);
                    contact.normal1 = cp.normal;
                    contact.rel1_cross = torque0;
                    bodies[sb0].apply_impulse(cp.normal * bodies[sb0].inv_mass, contact.ang_comp_a, contact.applied);

                    float vel1_dot = bullet_dot(contact.normal1, bodies[sb0].lin_vel + bodies[sb0].ext_force) +
                                     bullet_dot(contact.rel1_cross, bodies[sb0].ang_vel + bodies[sb0].ext_torque);

                    contact.ang_comp_b = Vec3::zero();
                    contact.normal2 = cp.normal * -1.0f;
                    contact.rel2_cross = Vec3::zero();

                    contact.jac_inv = (fabsf(denom0) < 1e-12f) ? 0.0f : CONTACT_SOR / denom0;

                    float rel_vel = cp.normal.dot(vel0);
                    float rest = fmaxf(0.0f, FullSolverConstraint::restitution_curve(rel_vel, cp.restitution));

                    float pen = cp.distance;
                    float pos_err = (pen > 0) ? 0.0f : -pen * CONTACT_ERP2 * inv_dt;
                    float vel_err = rest - vel1_dot;

                    float pen_impulse = pos_err * contact.jac_inv;
                    float vel_impulse = vel_err * contact.jac_inv;

                    if (pen > CONTACT_SPLIT_PENETRATION_THRESH) {
                        contact.rhs = pen_impulse + vel_impulse;
                        contact.rhs_pen = 0;
                    } else {
                        contact.rhs = vel_impulse;
                        contact.rhs_pen = pen_impulse;
                    }
                } else {
                    RigidBody& b1 = rbs[mf.body1_idx];
                    contact.setup_contact(b0, b1, bodies[sb0], bodies[sb1], rel1, rel2, cp, dt);
                }

                SolverBody& sba = bodies[sb0];
                SolverBody& sbb = bodies[sb1];
                Vec3 vel = sba.get_vel_at_no_delta(rel1) - sbb.get_vel_at_no_delta(rel2);
                float rel_vel = cp.normal.dot(vel);
                Vec3 lat_dir = vel - cp.normal * rel_vel;
                float lat_len_sq = lat_dir.length_squared();
                lat_dir = (lat_len_sq > SIMD_EPSILON) ? lat_dir * rsqrtf(fmaxf(lat_len_sq, 1e-12f)) : plane_space_1(cp.normal);

                FullSolverConstraint& fric = friction[num_friction++];
                fric.body_a = sb0;
                fric.body_b = sb1;
                fric.friction_idx = contact_idx;
                fric.lower = -cp.friction;
                fric.upper = cp.friction;
                fric.friction = cp.friction;
                fric.applied = 0;

                if (is_static_collision) {
                    fric.normal1 = lat_dir;
                    fric.rel1_cross = rel1.cross(lat_dir);
                    fric.ang_comp_a = b0.inv_inertia_world * fric.rel1_cross;
                    Vec3 fvec0 = fric.ang_comp_a.cross(rel1);
                    float f_denom3 = b0.inv_mass + lat_dir.dot(fvec0);
                    fric.jac_inv = (fabsf(f_denom3) < 1e-12f) ? 0.0f : CONTACT_SOR / f_denom3;

                    fric.normal2 = lat_dir * -1.0f;
                    fric.rel2_cross = Vec3::zero();
                    fric.ang_comp_b = Vec3::zero();

                    float f_vel1_dot = bullet_dot(fric.normal1, sba.lin_vel + sba.ext_force) +
                                       bullet_dot(fric.rel1_cross, sba.ang_vel);
                    fric.rhs = -f_vel1_dot * fric.jac_inv;
                } else {
                    RigidBody& b1 = rbs[mf.body1_idx];
                    fric.setup_friction(b0, b1, sba, sbb, rel1, rel2, lat_dir);
                }
            }
        }

        if (special.num_collisions > 0) {
            RigidBody& body = rbs[special.object_index];
            get_or_init_body(body, dt);
            convert_contact_special(body, dt);
            special.reset();
        }
    }

    RS_INLINE void solve_split() {
        for (int iter = 0; iter < CONTACT_NUM_ITERATIONS; iter++) {
            bool any = false;
            for (int i = 0; i < num_contacts; i++) {
                FullSolverConstraint& c = contacts[i];
                float res = c.resolve_split(bodies[c.body_a], bodies[c.body_b]);
                if (res * res > 1e-8f) any = true;
            }
            if (!any) break;
        }
    }

    RS_INLINE void solve_iterations() {
        solve_split();

        for (int iter = 0; iter < CONTACT_NUM_ITERATIONS; iter++) {
            float residual = 0;

            for (int i = 0; i < num_contacts; i++) {
                FullSolverConstraint& c = contacts[i];
                if (c.is_special)
                    continue;
                float r = c.resolve_lower(bodies[c.body_a], bodies[c.body_b]);
                residual = fmaxf(residual, r * r);
            }

            for (int i = 0; i < num_friction; i++) {
                FullSolverConstraint& f = friction[i];
                float total = contacts[f.friction_idx].applied;
                if (total <= 0) continue;
                float limit = f.friction * total;
                f.lower = -limit;
                f.upper = limit;

                float r = f.resolve_generic(bodies[f.body_a], bodies[f.body_b]);
                residual = fmaxf(residual, r * r);
            }

            if (residual < 1e-8f) break;
        }
    }

    RS_INLINE void write_back(RigidBody* rbs, float dt) {
        for (int i = 0; i < num_bodies; i++) {
            SolverBody& sb = bodies[i];
            if (sb.original_idx < 0) continue;

            RigidBody& rb = rbs[sb.original_idx];

            sb.lin_vel = sb.lin_vel + sb.delta_lin;
            sb.ang_vel = sb.ang_vel + sb.delta_ang;

            if (sb.push_vel.length_squared() > 0 || sb.turn_vel.length_squared() > 0) {
                if (rb.collision.no_rot) {
                    integrate_transform_no_rot(sb.world_transform, sb.push_vel, dt);
                } else {
                    integrate_transform(sb.world_transform, sb.push_vel, sb.turn_vel * CONTACT_SPLIT_TURN_ERP, dt);
                }
            }

            rb.linear_vel = sb.lin_vel + sb.ext_force;
            rb.angular_vel = sb.ang_vel + sb.ext_torque;
            rb.collision.transform = sb.world_transform;
        }
    }

    RS_INLINE void solve(RigidBody* rbs, int num_rbs, PersistentManifold* manifolds, int num_manifolds, float dt) {
        setup(rbs, num_rbs, manifolds, num_manifolds, dt);
        solve_iterations();
        write_back(rbs, dt);
    }
};
