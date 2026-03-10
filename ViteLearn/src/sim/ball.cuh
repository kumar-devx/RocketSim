#pragma once

#include "constants.cuh"
#include "../dynamics/body.cuh"

struct BallState {
    Vec3 pos;
    Mat3 rot;
    Vec3 vel;
    Vec3 ang_vel;

    RS_INLINE BallState() {
        pos = Vec3(0, 0, ball::REST_Z);
        rot = Mat3::identity();
        vel = Vec3::zero();
        ang_vel = Vec3::zero();
    }
};

struct Ball {
    BallState state;
    RigidBody body;
    Vec3 velocity_impulse_cache;
    float radius;
    long long last_extra_hit_tick;

    RS_INLINE Ball() : radius(ball::RADIUS), velocity_impulse_cache(Vec3::zero()), last_extra_hit_tick(-2) {}

    RS_INLINE void init_body() {
        float phys_radius = physics::to_phys(radius);
        float mass = ball::MASS;
        float inertia = (2.0f / 5.0f) * mass * phys_radius * phys_radius;
        Vec3 local_inertia(inertia, inertia, inertia);

        SphereShape sphere(phys_radius);
        CollisionShape shape;
        shape.type = SHAPE_SPHERE;
        shape.sphere = sphere;

        Affine3 trans;
        trans.translation = physics::to_phys(state.pos);
        trans.matrix3 = state.rot;

        body.init(mass, local_inertia, trans, shape, ball::FRICTION, ball::RESTITUTION);
        body.linear_damping = ball::DRAG;
        body.angular_damping = 0.0f;
        body.collision.no_rot = false;
    }

    RS_INLINE void update_from_body() {
        state.pos = physics::from_phys(body.collision.transform.translation);
        state.rot = body.collision.transform.matrix3;
        state.vel = physics::from_phys(body.linear_vel);
        state.ang_vel = body.angular_vel;
    }

    RS_INLINE void sync_to_body() {
        body.collision.transform.translation = physics::to_phys(state.pos);
        body.collision.transform.matrix3 = state.rot;
        body.linear_vel = physics::to_phys(state.vel);
        body.angular_vel = state.ang_vel;
        body.update_inertia();

        if (state.vel.length_squared() > 0 || state.ang_vel.length_squared() > 0) {
            body.collision.state = ACTIVE;
        } else {
            body.collision.state = SLEEPING;
        }
    }

    RS_INLINE void finish_physics_tick(RigidBody& rb) {
        if (velocity_impulse_cache.length_squared() > 0) {
            rb.linear_vel = rb.linear_vel + velocity_impulse_cache;
            velocity_impulse_cache = Vec3::zero();
        }

        float max_speed_phys = physics::to_phys(ball::MAX_SPEED);
        if (rb.linear_vel.length_squared() > max_speed_phys * max_speed_phys) {
            rb.linear_vel = rb.linear_vel.normalized() * max_speed_phys;
        }

        if (rb.angular_vel.length_squared() > ball::MAX_ANG_SPEED * ball::MAX_ANG_SPEED) {
            rb.angular_vel = rb.angular_vel.normalized() * ball::MAX_ANG_SPEED;
        }

        state.vel = physics::from_phys(rb.linear_vel);
        state.ang_vel = rb.angular_vel;
        state.pos = physics::from_phys(rb.collision.transform.translation);
        state.rot = rb.collision.transform.matrix3;
    }
};
