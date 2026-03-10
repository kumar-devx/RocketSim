#pragma once

#include "body.cuh"
#include "constraint.cuh"
#include "../sim/constants.cuh"

constexpr int NUM_WHEELS = 4;
constexpr float ROLLING_FRICTION_SCALE = 113.73963f;
constexpr float BILATERAL_DAMPING = -0.2f;

RS_INLINE float get_jacobian_diagonal(
    const Mat3& world_a, const Vec3& rel_a, const Vec3& inv_inertia_a, float inv_mass_a,
    const Mat3& world_b, const Vec3& rel_b, const Vec3& inv_inertia_b, float inv_mass_b,
    const Vec3& axis
) {
    Vec3 a_j = world_a * rel_a.cross(axis);
    Vec3 b_j = world_b * rel_b.cross(axis * -1.0f);
    Vec3 min_a = inv_inertia_a * a_j;
    Vec3 min_b = inv_inertia_b * b_j;
    return inv_mass_a + min_a.dot(a_j) + inv_mass_b + min_b.dot(b_j);
}

RS_INLINE float resolve_single_bilateral(
    const RigidBody& b1, const RigidBody& b2,
    const Vec3& pos1, const Vec3& pos2,
    const Vec3& normal
) {
    Vec3 rel1 = pos1 - b1.collision.transform.translation;
    Vec3 rel2 = pos2 - b2.collision.transform.translation;
    Vec3 vel1 = b1.get_velocity_at(rel1);
    Vec3 vel2 = b2.get_velocity_at(rel2);
    Vec3 vel = vel1 - vel2;

    Mat3 w1 = b1.collision.transform.matrix3.transpose();
    Mat3 w2 = b2.collision.transform.matrix3.transpose();

    float jac = get_jacobian_diagonal(
        w1, rel1, b1.inv_inertia_local, b1.inv_mass,
        w2, rel2, b2.inv_inertia_local, b2.inv_mass,
        normal
    );
    float jac_inv = 1.0f / jac;
    float rel_vel = normal.dot(vel);

    return BILATERAL_DAMPING * rel_vel * jac_inv;
}

RS_INLINE float resolve_single_collision(
    const RigidBody& b1, const RigidBody& b2,
    const Vec3& contact_pos, const Vec3& contact_normal,
    float dt, float distance
) {
    Vec3 rel1 = contact_pos - b1.collision.transform.translation;
    Vec3 rel2 = contact_pos - b2.collision.transform.translation;
    Vec3 vel1 = b1.get_velocity_at(rel1);
    Vec3 vel2 = b2.get_velocity_at(rel2);
    Vec3 vel = vel1 - vel2;
    float rel_vel = contact_normal.dot(vel);

    float pos_err = CONTACT_ERP * -distance / dt;
    float vel_err = -rel_vel;
    float denom0 = b1.compute_impulse_denom(contact_pos, contact_normal);
    float denom1 = b2.compute_impulse_denom(contact_pos, contact_normal);
    float jac_inv = 1.0f / (denom0 + denom1);

    float pen_imp = pos_err * jac_inv;
    float vel_imp = vel_err * jac_inv;
    return fmaxf(0, pen_imp + vel_imp);
}

struct RaycastInfo {
    Vec3 contact_normal, contact_point;
    Vec3 hard_point, wheel_dir, wheel_axle;
    float suspension_length;
    int ground_object;
    bool in_contact, has_ground;

    RS_INLINE RaycastInfo() : suspension_length(0), ground_object(-1), in_contact(false), has_ground(false) {}
};

struct WheelInfo {
    RaycastInfo raycast;
    Affine3 transform;
    Vec3 connection_cs, direction_cs, axle_cs;
    float rest_length, radius;
    float engine_force, brake;
    float inv_contact_dot, suspension_rel_vel, suspension_force;
    float steer_angle;
    Vec3 vel_at_contact, impulse;
    float lat_friction, long_friction;
    float suspension_scale, extra_pushback;
    bool in_world_contact;
    float debug_side_impulse, debug_rolling_friction;

    RS_INLINE WheelInfo() : rest_length(0), radius(0), engine_force(0), brake(0),
        inv_contact_dot(0), suspension_rel_vel(0), suspension_force(0), steer_angle(0),
        lat_friction(0), long_friction(0), suspension_scale(0), extra_pushback(0), in_world_contact(false),
        debug_side_impulse(0), debug_rolling_friction(0) {}

    RS_INLINE void init(const Vec3& conn, const Vec3& dir, const Vec3& axle, float rest, float rad) {
        connection_cs = conn; direction_cs = dir; axle_cs = axle;
        rest_length = rest; radius = rad;
    }

    RS_INLINE void update_transform_ws(const Affine3& chassis) {
        in_world_contact = false;
        raycast.in_contact = false;
        raycast.hard_point = chassis.transform_point(connection_cs);
        raycast.wheel_dir = chassis.matrix3 * direction_cs;
        raycast.wheel_axle = chassis.matrix3 * axle_cs;
    }

    RS_INLINE void update_transform(const Affine3& chassis) {
        update_transform_ws(chassis);
        Vec3 up = raycast.wheel_dir * -1.0f;
        Vec3 right = raycast.wheel_axle;
        Vec3 fwd = up.cross(right).normalized();

        Quat steer_q = quat_from_axis_angle(up, steer_angle);
        Mat3 steer_m = bullet_mat3_from_quat(steer_q);
        Mat3 basis = Mat3(fwd, right * -1.0f, up);

        transform.matrix3 = steer_m * basis;
        transform.translation = raycast.hard_point + direction_cs * raycast.suspension_length;
    }

    RS_INLINE void prepare_raycast(const Affine3& chassis, Vec3& src, Vec3& tgt, float& travel) {
        update_transform_ws(chassis);
        travel = vehicle::MAX_SUSPENSION_TRAVEL * physics::SCALE;
        float ray_len = rest_length + travel + radius - vehicle::SUSPENSION_SUBTRACTION;
        src = raycast.hard_point;
        tgt = src + raycast.wheel_dir * ray_len;
        raycast.contact_point = tgt;
        raycast.ground_object = -1;
        raycast.has_ground = false;
    }

    RS_INLINE void reset_suspension(float travel) {
        raycast.suspension_length = rest_length + travel;
        suspension_rel_vel = 0;
        raycast.contact_normal = raycast.wheel_dir * -1.0f;
        inv_contact_dot = 1.0f;
        extra_pushback = 0;
    }

    RS_INLINE void update_suspension(RigidBody& chassis, float dt) {
        if (!raycast.in_contact) { suspension_force = 0; return; }

        float force = (rest_length - raycast.suspension_length) * vehicle::SUSPENSION_STIFFNESS * inv_contact_dot;
        float damp = (suspension_rel_vel < 0) ? vehicle::DAMPING_COMPRESSION : vehicle::DAMPING_RELAXATION;
        suspension_force = (force - damp * suspension_rel_vel) * suspension_scale;
        suspension_force = fmaxf(suspension_force, 0);

        if (suspension_force == 0) return;

        float scale = suspension_force * dt + extra_pushback;
        Vec3 offset = raycast.contact_point - chassis.collision.transform.translation;

        Vec3 impulse_vec = raycast.contact_normal * scale;
        chassis.apply_impulse(impulse_vec, offset);
    }

    RS_INLINE void apply_friction(RigidBody& chassis, float dt) {
        if (impulse.x == 0 && impulse.y == 0 && impulse.z == 0) return;

        const Affine3& t = chassis.collision.transform;
        Vec3 offset = raycast.contact_point - t.translation;
        float up_dot = t.matrix3.z_axis.dot(offset);
        Vec3 rel = offset - t.matrix3.z_axis * up_dot;
        chassis.apply_impulse(impulse * dt, rel);
    }

    RS_INLINE void apply_raycast(const RigidBody& chassis, const RigidBody& ground,
        int ground_idx, bool ground_static,
        const Vec3& hit_point, const Vec3& hit_normal, float travel, float dt) {
        raycast.contact_point = hit_point;
        raycast.contact_normal = hit_normal;
        raycast.in_contact = true;
        in_world_contact = ground_static;
        raycast.ground_object = ground_idx;

        Vec3 up = chassis.collision.transform.matrix3.z_axis;
        float wheel_trace = (raycast.hard_point - raycast.contact_point).dot(up);
        raycast.suspension_length = wheel_trace - radius;

        float min_len = rest_length - travel;
        float max_len = rest_length + travel;
        raycast.suspension_length = fmaxf(min_len, fminf(max_len, raycast.suspension_length));

        Vec3 rel = raycast.contact_point - chassis.collision.transform.translation;
        vel_at_contact = chassis.get_velocity_at(rel);

        float proj_vel = raycast.contact_normal.dot(vel_at_contact);
        float denom = raycast.contact_normal.dot(up);

        if (denom > 0.1f) {
            float inv = 1.0f / denom;
            suspension_rel_vel = proj_vel * inv;
            inv_contact_dot = inv;
        } else {
            suspension_rel_vel = 0;
            inv_contact_dot = 10.0f;
        }

        if (in_world_contact) {
            float ray_pushback_thresh = rest_length + radius - vehicle::SUSPENSION_SUBTRACTION;
            if (wheel_trace < ray_pushback_thresh) {
                float wheel_trace_delta = wheel_trace - ray_pushback_thresh;
                float collision_result = resolve_single_collision(
                    chassis, ground, hit_point, hit_normal, dt, wheel_trace_delta
                );
                extra_pushback = collision_result / (float)NUM_WHEELS;
            }
        }
    }

    RS_INLINE void calc_friction_impulses(const RigidBody& chassis, const RigidBody& ground,
        float friction_scale, float dt) {
        if (raycast.ground_object < 0) {
            impulse = Vec3::zero();
            debug_side_impulse = 0;
            debug_rolling_friction = 0;
            return;
        }

        Vec3 axle_dir = transform.matrix3.y_axis;
        Vec3 surf_normal = raycast.contact_normal;
        float proj = axle_dir.dot(surf_normal);
        axle_dir = axle_dir - surf_normal * proj;
        axle_dir = axle_dir.normalize_or_zero();

        Vec3 forward_dir = surf_normal.cross(axle_dir).normalize_or_zero();

        float side_impulse = resolve_single_bilateral(
            chassis, ground,
            raycast.contact_point, raycast.contact_point, axle_dir
        );

        float rolling_friction;
        if (engine_force == 0) {
            if (brake == 0) {
                rolling_friction = 0;
            } else {
                Vec3 contact = raycast.contact_point;
                Vec3 car_rel = contact - chassis.collision.transform.translation;

                Vec3 v1 = chassis.get_velocity_at(car_rel);
                Vec3 v2 = ground.get_velocity_at(car_rel);
                Vec3 contact_vel = v1 - v2;
                float rel_vel = contact_vel.dot(forward_dir);

                if (dt > 1.0f / 80.0f) {
                    float thresh = 0.8f - (1.0f / (dt * 150.0f));
                    if (fabsf(rel_vel) < thresh) rel_vel = 0;
                }

                rolling_friction = -rel_vel * ROLLING_FRICTION_SCALE;
                rolling_friction = fmaxf(-brake, fminf(brake, rolling_friction));
            }
        } else {
            rolling_friction = -engine_force / friction_scale;
        }

        debug_side_impulse = side_impulse;
        debug_rolling_friction = rolling_friction;

        Vec3 total = forward_dir * rolling_friction * long_friction + axle_dir * side_impulse * lat_friction;
        impulse = total * friction_scale;
    }
};

struct Vehicle {
    WheelInfo wheels[NUM_WHEELS];
    int chassis_idx;
    int num_wheels;

    RS_INLINE Vehicle() : chassis_idx(0), num_wheels(0) {}

    RS_INLINE void add_wheel(const Vec3& conn, const Vec3& dir, const Vec3& axle, float rest, float rad) {
        if (num_wheels < NUM_WHEELS) {
            wheels[num_wheels].init(conn, dir, axle, rest, rad);
            num_wheels++;
        }
    }

    RS_INLINE Vec3 get_contact_up(const RigidBody& chassis) const {
        Vec3 sum = Vec3::zero();
        for (int i = 0; i < num_wheels; i++) {
            if (wheels[i].raycast.in_contact) sum = sum + wheels[i].raycast.contact_normal;
        }
        return (sum.x == 0 && sum.y == 0 && sum.z == 0) ? chassis.up() : sum.normalized();
    }
};
