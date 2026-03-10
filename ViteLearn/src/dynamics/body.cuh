#pragma once

#include "../collision/shapes.cuh"

enum ActivationState { ACTIVE, SLEEPING, DISABLED };
enum CollisionFlags { CF_STATIC = 1, CF_NO_RESPONSE = 2, CF_CUSTOM_MATERIAL = 4 };

struct CollisionObject {
    Affine3 transform;
    Affine3 interp_transform;
    Vec3 interp_linear_vel;
    Vec3 interp_angular_vel;
    Vec3 hitbox_offset;
    CollisionShape shape;
    unsigned char flags;
    int world_idx;
    int companion_id;
    int user_type;
    int user_id;
    float friction, restitution;
    float contact_processing_threshold;
    ActivationState state;
    float deactivation_time;
    bool no_rot;

    RS_INLINE CollisionObject() : flags(0), world_idx(0), companion_id(-1), user_type(0), user_id(-1),
        friction(0), restitution(0), contact_processing_threshold(1e18f), state(ACTIVE), deactivation_time(0), no_rot(false) {}
    RS_INLINE bool is_static() const { return flags & CF_STATIC; }
    RS_INLINE bool is_active() const { return state == ACTIVE; }
    RS_INLINE Affine3 get_hitbox_transform() const {
        Affine3 result = transform;
        result.translation = result.translation + transform.matrix3 * hitbox_offset;
        return result;
    }
};

struct RigidBody {
    CollisionObject collision;
    Mat3 inv_inertia_world;
    Vec3 linear_vel, angular_vel;
    Vec3 inv_inertia_local;
    Vec3 total_force, total_torque;
    Vec3 gravity;
    Vec3 gravity_accel;
    float inv_mass;
    float linear_damping, angular_damping;
    float linear_sleep_thresh, angular_sleep_thresh;
    bool can_sleep;

    RS_INLINE RigidBody() : inv_mass(0), linear_damping(0), angular_damping(0), linear_sleep_thresh(0), angular_sleep_thresh(1), can_sleep(false) {}

    RS_INLINE void init(float mass, const Vec3& inertia, const Affine3& trans, const CollisionShape& shape, float fric, float rest, bool can_sleep_ = false) {
        collision.transform = trans;
        collision.shape = shape;
        collision.friction = fric;
        collision.restitution = rest;
        collision.flags = (mass == 0) ? CF_STATIC : 0;

        inv_mass = (mass == 0) ? 0 : 1.0f / mass;
        inv_inertia_local = Vec3(
            inertia.x == 0 ? 0 : 1.0f / inertia.x,
            inertia.y == 0 ? 0 : 1.0f / inertia.y,
            inertia.z == 0 ? 0 : 1.0f / inertia.z
        );
        can_sleep = can_sleep_;
        update_inertia();
    }

    RS_INLINE void update_inertia() {
        Mat3 basis = collision.transform.matrix3;
        Mat3 scaled;
        scaled.x_axis = basis.x_axis * inv_inertia_local.x;
        scaled.y_axis = basis.y_axis * inv_inertia_local.y;
        scaled.z_axis = basis.z_axis * inv_inertia_local.z;
        inv_inertia_world = scaled * basis.transpose();
    }

    RS_INLINE void set_gravity(const Vec3& g) {
        gravity_accel = g;
        gravity = g * (inv_mass > 0 ? 1.0f / inv_mass : 0);
    }
    RS_INLINE void apply_gravity() { total_force = total_force + gravity; }
    RS_INLINE void apply_central_force(const Vec3& f) {
        total_force = total_force + f;
    }
    RS_INLINE void apply_torque(const Vec3& t) {
        total_torque = total_torque + t;
    }
    RS_INLINE void apply_central_impulse(const Vec3& imp) { linear_vel = linear_vel + imp * inv_mass; }
    RS_INLINE void apply_torque_impulse(const Vec3& t) { angular_vel = angular_vel + inv_inertia_world * t; }
    RS_INLINE void apply_impulse(const Vec3& imp, const Vec3& rel) { apply_central_impulse(imp); apply_torque_impulse(rel.cross(imp)); }
    RS_INLINE void clear_forces() { total_force = Vec3::zero(); total_torque = Vec3::zero(); }

    RS_INLINE Vec3 get_velocity_at(const Vec3& rel) const { return linear_vel + angular_vel.cross(rel); }
    RS_INLINE Vec3 up() const { return collision.transform.matrix3.z_axis; }
    RS_INLINE Vec3 forward() const { return collision.transform.matrix3.x_axis; }
    RS_INLINE float get_forward_speed() const { return forward().dot(linear_vel); }

    RS_INLINE void apply_damping(float dt) {
        if (linear_damping != 0.0f)
            linear_vel = linear_vel * (1.0f - linear_damping * dt);
        if (angular_damping != 0.0f)
            angular_vel = angular_vel * (1.0f - angular_damping * dt);
    }

    RS_INLINE Affine3 predict_transform(float dt) const {
        Affine3 t = collision.transform;
        integrate_transform(t, linear_vel, collision.no_rot ? Vec3::zero() : angular_vel, dt);
        return t;
    }

    RS_INLINE void set_transform(const Affine3& t) {
        collision.transform = t;
        update_inertia();
    }

    RS_INLINE void set_centre_of_mass_trans(const Affine3& xform) {
        collision.interp_transform = xform;
        collision.interp_linear_vel = linear_vel;
        collision.interp_angular_vel = angular_vel;
        collision.transform = xform;
        update_inertia();
    }

    RS_INLINE float compute_impulse_denom(const Vec3& pos, const Vec3& normal) const {
        Vec3 r = pos - collision.transform.translation;
        Vec3 c = r.cross(normal);
        Vec3 v = (inv_inertia_world * c).cross(r);
        return inv_mass + normal.dot(v);
    }

    RS_INLINE void set_activation_state(ActivationState new_state) {
        if (collision.state != DISABLED) collision.state = new_state;
    }

    RS_INLINE void update_activation_state(float dt) {
        if (!can_sleep) {
            set_activation_state(ACTIVE);
            return;
        }
        float thresh_lin_sq = linear_sleep_thresh * linear_sleep_thresh;
        float thresh_ang_sq = angular_sleep_thresh * angular_sleep_thresh;
        bool within = (linear_vel.length_squared() < thresh_lin_sq) && (angular_vel.length_squared() < thresh_ang_sq);
        set_activation_state(within ? SLEEPING : ACTIVE);
    }
};
