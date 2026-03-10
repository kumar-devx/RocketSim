#pragma once

#include "constants.cuh"
#include "../dynamics/vehicle.cuh"

constexpr int CAR_TYPE_OCTANE = 0;

struct WheelPairConfig {
    float wheel_radius;
    float suspension_rest_length;
    Vec3 connection_offset;
};

struct CarConfig {
    Vec3 hitbox_size;
    Vec3 hitbox_offset;
    WheelPairConfig front_wheels;
    WheelPairConfig back_wheels;
    bool three_wheels;
    float dodge_deadzone;
};

RS_INLINE CarConfig make_car_config(int index) {
    CarConfig cfg;
    cfg.three_wheels = (index == 6);
    cfg.dodge_deadzone = 0.5f;
    switch (index) {
        case 0:
            cfg.hitbox_size = Vec3(120.507f, 86.6994f, 38.6591f);
            cfg.hitbox_offset = Vec3(13.8757f, 0.0f, 20.755f);
            cfg.front_wheels.wheel_radius = 12.5f;
            cfg.front_wheels.suspension_rest_length = 38.755f;
            cfg.front_wheels.connection_offset = Vec3(51.25f, 25.90f, 20.755f);
            cfg.back_wheels.wheel_radius = 15.0f;
            cfg.back_wheels.suspension_rest_length = 37.055f;
            cfg.back_wheels.connection_offset = Vec3(-33.75f, 29.50f, 20.755f);
            break;
        case 1:
            cfg.hitbox_size = Vec3(130.427f, 85.7799f, 33.8f);
            cfg.hitbox_offset = Vec3(9.0f, 0.0f, 15.75f);
            cfg.front_wheels.wheel_radius = 12.0f;
            cfg.front_wheels.suspension_rest_length = 33.95f;
            cfg.front_wheels.connection_offset = Vec3(50.30f, 31.10f, 15.75f);
            cfg.back_wheels.wheel_radius = 13.5f;
            cfg.back_wheels.suspension_rest_length = 33.85f;
            cfg.back_wheels.connection_offset = Vec3(-34.75f, 33.00f, 15.75f);
            break;
        case 2:
            cfg.hitbox_size = Vec3(131.32f, 87.1704f, 31.8944f);
            cfg.hitbox_offset = Vec3(9.00857f, 0.0f, 12.0942f);
            cfg.front_wheels.wheel_radius = 12.5f;
            cfg.front_wheels.suspension_rest_length = 31.9242f;
            cfg.front_wheels.connection_offset = Vec3(49.97f, 27.80f, 12.0942f);
            cfg.back_wheels.wheel_radius = 17.0f;
            cfg.back_wheels.suspension_rest_length = 27.9242f;
            cfg.back_wheels.connection_offset = Vec3(-35.43f, 20.28f, 12.0942f);
            break;
        case 3:
            cfg.hitbox_size = Vec3(133.992f, 83.021f, 32.8f);
            cfg.hitbox_offset = Vec3(12.5f, 0.0f, 11.75f);
            cfg.front_wheels.wheel_radius = 13.5f;
            cfg.front_wheels.suspension_rest_length = 29.7f;
            cfg.front_wheels.connection_offset = Vec3(51.50f, 26.67f, 11.75f);
            cfg.back_wheels.wheel_radius = 15.0f;
            cfg.back_wheels.suspension_rest_length = 29.666f;
            cfg.back_wheels.connection_offset = Vec3(-35.75f, 35.00f, 11.75f);
            break;
        case 4:
            cfg.hitbox_size = Vec3(129.519f, 84.6879f, 36.6591f);
            cfg.hitbox_offset = Vec3(13.8757f, 0.0f, 20.755f);
            cfg.front_wheels.wheel_radius = 12.5f;
            cfg.front_wheels.suspension_rest_length = 38.755f;
            cfg.front_wheels.connection_offset = Vec3(51.25f, 25.90f, 20.755f);
            cfg.back_wheels.wheel_radius = 15.0f;
            cfg.back_wheels.suspension_rest_length = 37.055f;
            cfg.back_wheels.connection_offset = Vec3(-34.00f, 29.50f, 20.755f);
            break;
        case 5:
            cfg.hitbox_size = Vec3(123.22f, 79.2103f, 44.1591f);
            cfg.hitbox_offset = Vec3(11.3757f, 0.0f, 21.505f);
            cfg.front_wheels.wheel_radius = 15.0f;
            cfg.front_wheels.suspension_rest_length = 39.505f;
            cfg.front_wheels.connection_offset = Vec3(51.25f, 25.90f, 21.505f);
            cfg.back_wheels.wheel_radius = 15.0f;
            cfg.back_wheels.suspension_rest_length = 39.105f;
            cfg.back_wheels.connection_offset = Vec3(-33.75f, 29.50f, 21.505f);
            break;
        default:
            cfg.hitbox_size = Vec3(120.641f, 86.8334f, 38.7931f);
            cfg.hitbox_offset = Vec3(13.8757f, 0.0f, 15.0f);
            cfg.front_wheels.wheel_radius = 12.5f;
            cfg.front_wheels.suspension_rest_length = 33.0f;
            cfg.front_wheels.connection_offset = Vec3(51.25f, 5.000f, 15.000f);
            cfg.back_wheels.wheel_radius = 15.0f;
            cfg.back_wheels.suspension_rest_length = 31.3f;
            cfg.back_wheels.connection_offset = Vec3(-33.75f, 29.50f, 15.000f);
            break;
    }
    return cfg;
}

struct CarControls {
    float throttle;
    float steer;
    float pitch;
    float yaw;
    float roll;
    bool jump;
    bool boost;
    bool handbrake;

    RS_INLINE CarControls() : throttle(0), steer(0), pitch(0), yaw(0), roll(0),
        jump(false), boost(false), handbrake(false) {}

    RS_INLINE CarControls clamp() const {
        CarControls c;
        c.throttle = fmaxf(-1.0f, fminf(1.0f, throttle));
        c.steer = fmaxf(-1.0f, fminf(1.0f, steer));
        c.pitch = fmaxf(-1.0f, fminf(1.0f, pitch));
        c.yaw = fmaxf(-1.0f, fminf(1.0f, yaw));
        c.roll = fmaxf(-1.0f, fminf(1.0f, roll));
        c.jump = jump;
        c.boost = boost;
        c.handbrake = handbrake;
        return c;
    }
};

struct CarContact {
    unsigned long long other_car_id;
    float cooldown_timer;
};

struct BallHitInfo {
    bool is_valid;
    Vec3 relative_pos;
    Vec3 ball_pos;
    Vec3 extra_hit_vel;
    unsigned long long tick_count_when_hit;
    unsigned long long tick_count_when_extra_impulse_applied;
};

struct CarState {
    Vec3 pos;
    Mat3 rot;
    Vec3 vel;
    Vec3 ang_vel;

    CarControls controls;
    CarControls prev_controls;

    bool is_on_ground;
    bool wheels_contact[4];

    bool has_jumped;
    bool has_double_jumped;
    bool has_flipped;

    Vec3 flip_rel_torque;
    float jump_time;
    float flip_time;
    bool is_flipping;
    bool is_jumping;

    float air_time;
    float air_time_since_jump;

    float boost_amount;
    float time_since_boosted;
    bool is_boosting;
    float boosting_time;

    bool is_supersonic;
    float supersonic_time;

    float handbrake_val;

    bool is_auto_flipping;
    float auto_flip_timer;
    float auto_flip_torque_scale;

    Vec3 world_contact_normal;
    bool has_world_contact;

    bool has_car_contact;
    CarContact car_contact;
    float bump_cooldown_timer;

    bool is_demoed;
    float demo_respawn_timer;

    bool has_ball_hit;
    BallHitInfo ball_hit_info;

    RS_INLINE CarState() {
        pos = Vec3(0, 0, car::spawn::SPAWN_Z);
        rot = Mat3::identity();
        vel = Vec3::zero();
        ang_vel = Vec3::zero();

        is_on_ground = true;
        for (int i = 0; i < 4; i++) wheels_contact[i] = false;

        has_jumped = false;
        has_double_jumped = false;
        has_flipped = false;

        flip_rel_torque = Vec3::zero();
        jump_time = 0;
        flip_time = 0;
        is_flipping = false;
        is_jumping = false;

        air_time = 0;
        air_time_since_jump = 0;

        boost_amount = car::boost::SPAWN_AMOUNT;
        time_since_boosted = 0;
        is_boosting = false;
        boosting_time = 0;

        is_supersonic = false;
        supersonic_time = 0;

        handbrake_val = 0;

        is_auto_flipping = false;
        auto_flip_timer = 0;
        auto_flip_torque_scale = 0;

        world_contact_normal = Vec3::zero();
        has_world_contact = false;

        has_car_contact = false;
        bump_cooldown_timer = 0;

        is_demoed = false;
        demo_respawn_timer = 0;

        has_ball_hit = false;
    }

    RS_INLINE bool has_flip_or_jump() const {
        return is_on_ground ||
            (!has_flipped && !has_double_jumped &&
             air_time_since_jump < car::jump::DOUBLEJUMP_MAX_DELAY);
    }

    RS_INLINE bool has_flip_reset() const {
        return !is_on_ground && has_flip_or_jump() && !has_jumped;
    }

    RS_INLINE bool got_flip_reset() const {
        return !is_on_ground && !has_jumped;
    }
};

struct Car {
    CarState state;
    CarConfig config;
    Vehicle vehicle;
    RigidBody body;
    Vec3 velocity_impulse_cache;
    int id;
    int team;
    int body_idx;

    RS_INLINE Car() : id(0), team(0), body_idx(-1), velocity_impulse_cache(Vec3::zero()) {
        config = make_car_config(CAR_TYPE_OCTANE);
    }

    RS_INLINE RigidBody& get_body(RigidBody* bodies) { return bodies[body_idx]; }

    RS_INLINE void init_vehicle() {
        Vec3 wheel_dir(0, 0, -1);
        Vec3 wheel_axle(0, -1, 0);

        Vec3 front = physics::to_phys(config.front_wheels.connection_offset);
        Vec3 back = physics::to_phys(config.back_wheels.connection_offset);
        float front_rest = physics::to_phys(config.front_wheels.suspension_rest_length - vehicle::MAX_SUSPENSION_TRAVEL);
        float back_rest = physics::to_phys(config.back_wheels.suspension_rest_length - vehicle::MAX_SUSPENSION_TRAVEL);
        float front_rad = physics::to_phys(config.front_wheels.wheel_radius);
        float back_rad = physics::to_phys(config.back_wheels.wheel_radius);

        vehicle.add_wheel(Vec3(front.x, front.y, front.z), wheel_dir, wheel_axle, front_rest, front_rad);
        vehicle.add_wheel(Vec3(front.x, -front.y, front.z), wheel_dir, wheel_axle, front_rest, front_rad);
        vehicle.add_wheel(Vec3(back.x, back.y, back.z), wheel_dir, wheel_axle, back_rest, back_rad);
        vehicle.add_wheel(Vec3(back.x, -back.y, back.z), wheel_dir, wheel_axle, back_rest, back_rad);

        vehicle.wheels[0].suspension_scale = vehicle::SUSPENSION_FORCE_FRONT;
        vehicle.wheels[1].suspension_scale = vehicle::SUSPENSION_FORCE_FRONT;
        vehicle.wheels[2].suspension_scale = vehicle::SUSPENSION_FORCE_BACK;
        vehicle.wheels[3].suspension_scale = vehicle::SUSPENSION_FORCE_BACK;
    }

    RS_INLINE void update_from_body(const RigidBody& world_body) {
        state.pos = physics::from_phys(world_body.collision.transform.translation);
        state.rot = world_body.collision.transform.matrix3;
        state.vel = physics::from_phys(world_body.linear_vel);
        state.ang_vel = world_body.angular_vel;
    }

    RS_INLINE void sync_to_body() {
        body.collision.transform.translation = physics::to_phys(state.pos);
        body.collision.transform.matrix3 = state.rot;
        body.linear_vel = physics::to_phys(state.vel);
        body.angular_vel = state.ang_vel;
        body.update_inertia();
        velocity_impulse_cache = Vec3::zero();
    }

    RS_INLINE int count_wheel_contacts() const {
        int count = 0;
        for (int i = 0; i < 4; i++) if (state.wheels_contact[i]) count++;
        return count;
    }

    RS_INLINE void update_wheel_contacts() {
        for (int i = 0; i < vehicle.num_wheels; i++) {
            state.wheels_contact[i] = vehicle.wheels[i].raycast.in_contact;
        }
        state.is_on_ground = count_wheel_contacts() >= 3;
    }

    RS_INLINE void update_wheels(float dt, float fwd_speed, RigidBody* bodies) {
        CarState& s = state;
        CarControls& c = s.controls;
        int num_contact = count_wheel_contacts();

        if (c.handbrake) {
            s.handbrake_val += car::drive::POWERSLIDE_RISE_RATE * dt;
        } else {
            s.handbrake_val -= car::drive::POWERSLIDE_FALL_RATE * dt;
        }
        s.handbrake_val = fmaxf(0.0f, fminf(1.0f, s.handbrake_val));

        float real_brake = 0.0f;
        float real_throttle = (c.boost && s.boost_amount > 0) ? 1.0f : c.throttle;
        float abs_fwd_speed = fabsf(fwd_speed);
        float engine_throttle = real_throttle;

        if (!c.handbrake) {
            if (fabsf(real_throttle) >= car::drive::THROTTLE_DEADZONE) {
                if (abs_fwd_speed > car::drive::STOPPING_FORWARD_VEL &&
                    ((real_throttle > 0) != (fwd_speed > 0))) {
                    real_brake = 1.0f;
                    if (abs_fwd_speed > car::drive::BRAKING_NO_THROTTLE_THRESH) engine_throttle = 0.0f;
                }
            } else {
                engine_throttle = 0.0f;
                real_brake = (abs_fwd_speed < car::drive::STOPPING_FORWARD_VEL) ? 1.0f : car::drive::COASTING_BRAKE_FACTOR;
            }
        }

        float drive_scale = curves::eval_drive_torque_factor(abs_fwd_speed);
        if (num_contact < 3) drive_scale /= 4.0f;

        float drive_force = physics::to_phys(engine_throttle * car::drive::THROTTLE_TORQUE * drive_scale);
        float brake_force = physics::to_phys(real_brake * car::drive::BRAKE_TORQUE);

        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].engine_force = drive_force;
            vehicle.wheels[i].brake = brake_force;
        }

        float steer = curves::eval_steer_angle(abs_fwd_speed);
        if (s.handbrake_val != 0) steer += (curves::eval_powerslide_steer(abs_fwd_speed) - steer) * s.handbrake_val;
        steer *= c.steer;
        vehicle.wheels[0].steer_angle = steer;
        vehicle.wheels[1].steer_angle = steer;

        RigidBody& rb = get_body(bodies);
        for (int i = 0; i < vehicle.num_wheels; i++) {
            WheelInfo& w = vehicle.wheels[i];
            if (!w.raycast.in_contact) continue;

            Vec3 wheel_delta = w.raycast.hard_point - rb.collision.transform.translation;
            Vec3 cross_vec = physics::from_phys(rb.angular_vel.cross(wheel_delta) + rb.linear_vel);

            Vec3 lat_dir = w.transform.matrix3.y_axis;
            Vec3 long_dir = lat_dir.cross(w.raycast.contact_normal);
            float base_friction = fabsf(cross_vec.dot(lat_dir));
            float friction_input = (base_friction > 5.0f) ? base_friction / (fabsf(cross_vec.dot(long_dir)) + base_friction) : 0.0f;

            w.lat_friction = curves::eval_lat_friction(friction_input);
            w.long_friction = 1.0f;

            if (s.handbrake_val != 0) {
                w.lat_friction *= 1.0f - curves::HANDBRAKE_LAT_FRICTION_FACTOR * s.handbrake_val;
                w.long_friction *= 1.0f + (curves::eval_handbrake_long_friction(friction_input) - 1.0f) * s.handbrake_val;
            }
            if (real_throttle == 0) {
                float non_sticky = curves::eval_non_sticky_friction(w.raycast.contact_normal.z);
                w.lat_friction *= non_sticky;
                w.long_friction *= non_sticky;
            }
        }

        bool has_world_contact = false;
        for (int i = 0; i < vehicle.num_wheels; i++) if (vehicle.wheels[i].in_world_contact) { has_world_contact = true; break; }

        if (has_world_contact) {
            Vec3 up_dir = vehicle.get_contact_up(rb);
            bool full_stick = real_throttle != 0 || abs_fwd_speed > car::drive::STOPPING_FORWARD_VEL;
            float sticky_scale = config.three_wheels ? 0.0f : 0.5f;
            if (full_stick) sticky_scale += 1.0f - fabsf(up_dir.z);
            rb.apply_central_force(up_dir * physics::to_phys(sticky_scale * GRAVITY_Z) * car::MASS);
        }
    }

    RS_INLINE void update_air_torque(bool update_control, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        Vec3 dir_pitch = s.rot.y_axis * -1.0f;
        Vec3 dir_yaw = s.rot.z_axis;
        Vec3 dir_roll = s.rot.x_axis * -1.0f;

        if (s.is_flipping) s.is_flipping = s.has_flipped && s.flip_time < car::flip::TORQUE_TIME;

        bool do_air = false;
        if (s.is_flipping && !(s.flip_rel_torque.x == 0 && s.flip_rel_torque.y == 0 && s.flip_rel_torque.z == 0)) {
            Vec3 rel_torque = s.flip_rel_torque;
            float pitch_scale = 1.0f;
            if (rel_torque.y != 0 && s.controls.pitch != 0 && ((rel_torque.y > 0) == (s.controls.pitch > 0))) {
                pitch_scale = 1.0f - fminf(1.0f, fabsf(s.controls.pitch));
                do_air = true;
            }
            rel_torque.y *= pitch_scale;
            Vec3 dodge_torque(rel_torque.x * car::flip::TORQUE_X, rel_torque.y * car::flip::TORQUE_Y, 0);
            Mat3 inv_inertia_inv = rb.inv_inertia_world.bullet_inverse();
            Vec3 rotated_torque = rb.collision.transform.matrix3 * dodge_torque;
            Vec3 final_torque = inv_inertia_inv * rotated_torque;
            rb.apply_torque(final_torque);
        } else {
            do_air = true;
        }

        do_air = do_air && !s.is_auto_flipping && update_control;
        if (do_air) {
            float pitch_scale = 1.0f;
            Vec3 torque = Vec3::zero();
            if (s.controls.pitch != 0 || s.controls.yaw != 0 || s.controls.roll != 0) {
                if (s.is_flipping || (s.has_flipped && s.flip_time < car::flip::TORQUE_TIME + car::flip::PITCHLOCK_EXTRA_TIME)) pitch_scale = 0.0f;
                torque = dir_pitch * s.controls.pitch * pitch_scale * car::air_control::TORQUE_X +
                         dir_yaw * s.controls.yaw * car::air_control::TORQUE_Y +
                         dir_roll * s.controls.roll * car::air_control::TORQUE_Z;
            }
            Vec3 ang = rb.angular_vel;
            Vec3 damping = dir_yaw * dir_yaw.dot(ang) * car::air_control::DAMPING_Y * (1.0f - fabsf(s.controls.yaw)) +
                           dir_pitch * dir_pitch.dot(ang) * car::air_control::DAMPING_X * (1.0f - fabsf(s.controls.pitch * pitch_scale)) +
                           dir_roll * dir_roll.dot(ang) * car::air_control::DAMPING_Z;
            Mat3 inv_inertia_inv = rb.inv_inertia_world.bullet_inverse();
            Vec3 final_torque = inv_inertia_inv * (torque - damping) * car::air_control::TORQUE_APPLY_SCALE;
            rb.apply_torque(final_torque);
        }
        if (s.controls.throttle != 0) {
            rb.apply_central_force(s.rot.x_axis * physics::to_phys(s.controls.throttle * car::drive::THROTTLE_AIR_ACCEL) * car::MASS);
        }
    }

    RS_INLINE void update_jump(float dt, bool jump_pressed, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        if (s.is_on_ground && s.is_jumping) {
            if (!(s.has_jumped && s.jump_time < car::jump::MIN_TIME + car::jump::RESET_TIME_PAD)) {
                s.has_jumped = false; s.jump_time = 0;
            }
        }
        if (s.is_jumping) {
            s.is_jumping = s.jump_time < car::jump::MIN_TIME || (s.controls.jump && s.jump_time < car::jump::MAX_TIME);
        } else if (s.is_on_ground && jump_pressed) {
            s.is_jumping = true; s.jump_time = 0;
            rb.apply_central_impulse(s.rot.z_axis * physics::to_phys(car::jump::IMMEDIATE_FORCE) * car::MASS);
        }
        if (s.is_jumping) {
            s.has_jumped = true;
            Vec3 force = s.rot.z_axis * car::jump::ACCEL * (s.jump_time < car::jump::MIN_TIME ? 0.62f : 1.0f);
            rb.apply_central_force(physics::to_phys(force) * car::MASS);
        }
        if (s.is_jumping || s.has_jumped) s.jump_time += dt;
    }

    RS_INLINE void update_flip(float dt, bool jump_pressed, float fwd_speed, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        if (s.is_on_ground) { s.has_double_jumped = s.has_flipped = false; s.air_time = s.air_time_since_jump = s.flip_time = 0; return; }
        s.air_time += dt;
        s.air_time_since_jump = (s.has_jumped && !s.is_jumping) ? s.air_time_since_jump + dt : 0;

        if (jump_pressed && s.air_time_since_jump < car::jump::DOUBLEJUMP_MAX_DELAY) {
            float input_mag = fabsf(s.controls.yaw) + fabsf(s.controls.pitch) + fabsf(s.controls.roll);
            bool is_flip = input_mag >= config.dodge_deadzone;
            if (!s.is_auto_flipping && !s.has_double_jumped && !s.has_flipped) {
                if (is_flip) {
                    s.flip_time = 0; s.has_flipped = s.is_flipping = true;
                    float speed_ratio = fabsf(fwd_speed) / car::MAX_SPEED;
                    Vec3 dodge_dir(-s.controls.pitch, s.controls.yaw + s.controls.roll, 0);
                    dodge_dir = (fabsf(dodge_dir.x) < 0.1f && fabsf(dodge_dir.y) < 0.1f) ? Vec3::zero() : dodge_dir.normalized();
                    s.flip_rel_torque = Vec3(-dodge_dir.y, dodge_dir.x, 0);
                    if (fabsf(dodge_dir.x) < 0.1f) dodge_dir.x = 0;
                    if (fabsf(dodge_dir.y) < 0.1f) dodge_dir.y = 0;
                    if (dodge_dir.length_squared() > SIMD_EPSILON * SIMD_EPSILON) {
                        bool backwards = (fabsf(fwd_speed) < 100.0f) ? (dodge_dir.x < 0) : ((dodge_dir.x > 0) != (fwd_speed > 0));
                        float max_x = backwards ? car::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE : car::flip::FORWARD_IMPULSE_MAX_SPEED_SCALE;
                        Vec3 init_vel = dodge_dir * car::flip::INITIAL_VEL_SCALE;
                        init_vel.x *= ((max_x - 1.0f) * speed_ratio) + 1.0f;
                        init_vel.y *= ((car::flip::SIDE_IMPULSE_MAX_SPEED_SCALE - 1.0f) * speed_ratio) + 1.0f;
                        if (backwards) init_vel.x *= car::flip::BACKWARD_IMPULSE_SCALE_X;
                        Vec3 fwd_2d = Vec3(s.rot.x_axis.x, s.rot.x_axis.y, 0).normalized();
                        rb.apply_central_impulse(physics::to_phys(fwd_2d * init_vel.x + Vec3(-fwd_2d.y, fwd_2d.x, 0) * init_vel.y) * car::MASS);
                    }
                } else {
                    rb.apply_central_impulse(s.rot.z_axis * physics::to_phys(car::jump::IMMEDIATE_FORCE) * car::MASS);
                    s.has_double_jumped = true;
                }
            }
        }
        if (s.is_flipping) {
            s.flip_time += dt;
            if (s.flip_time <= car::flip::TORQUE_TIME && s.flip_time >= car::flip::Z_DAMP_START && (rb.linear_vel.z < 0 || s.flip_time < car::flip::Z_DAMP_END)) {
                rb.linear_vel.z *= (1.0f - car::flip::Z_DAMP_120);
            }
        } else if (s.has_flipped) s.flip_time += dt;
    }

    RS_INLINE void update_auto_flip(float dt, bool jump_pressed, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        if (jump_pressed && s.has_world_contact && s.world_contact_normal.z > car::autoflip::NORM_Z_THRESH) {
            float roll = atan2f(s.rot.y_axis.z, s.rot.z_axis.z);
            float abs_roll = fabsf(roll);
            if (abs_roll > car::autoflip::ROLL_THRESH) {
                s.auto_flip_timer = car::autoflip::TIME * (abs_roll / PI_F);
                s.auto_flip_torque_scale = (roll > 0) ? 1.0f : -1.0f;
                s.is_auto_flipping = true;
                rb.apply_central_impulse(s.rot.z_axis * -1.0f * physics::to_phys(car::autoflip::IMPULSE) * car::MASS);
            }
        }
        if (s.is_auto_flipping) {
            if (s.auto_flip_timer <= 0) { s.is_auto_flipping = false; s.auto_flip_timer = 0; }
            else {
                rb.angular_vel = rb.angular_vel + s.rot.x_axis * car::autoflip::TORQUE * s.auto_flip_torque_scale * dt;
                s.auto_flip_timer -= dt;
            }
        }
    }

    RS_INLINE void update_auto_roll(int num_contact, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        Vec3 ground_up = (num_contact > 0) ? vehicle.get_contact_up(rb) : s.world_contact_normal;
        Vec3 ground_down = ground_up * -1.0f;
        Vec3 fwd = s.rot.x_axis;
        Vec3 right = s.rot.y_axis;

        Vec3 cross_right = ground_up.cross(fwd);
        Vec3 cross_fwd = ground_down.cross(cross_right);

        float right_factor = 1.0f - fmaxf(0.0f, fminf(1.0f, right.dot(cross_right)));
        float fwd_factor = 1.0f - fmaxf(0.0f, fminf(1.0f, fwd.dot(cross_fwd)));

        float right_dot = right.dot(ground_up);
        float fwd_dot = fwd.dot(ground_up);
        float right_sign = (right_dot < 0) ? -1.0f : 1.0f;
        float fwd_sign = (fwd_dot < 0) ? -1.0f : 1.0f;
        Vec3 torque_dir_right = fwd * -right_sign;
        Vec3 torque_dir_fwd = right * fwd_sign;

        Vec3 torque_right = torque_dir_right * right_factor;
        Vec3 torque_fwd = torque_dir_fwd * fwd_factor;

        rb.apply_central_force(ground_down * physics::to_phys(car::autoroll::FORCE) * car::MASS);
        rb.apply_torque(rb.inv_inertia_world.bullet_inverse() * (torque_fwd + torque_right) * car::autoroll::TORQUE);
    }

    RS_INLINE void update_boost(float dt, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        CarState& s = state;
        s.is_boosting = (s.boost_amount > 0) && (s.controls.boost || (s.is_boosting && s.boosting_time < car::boost::MIN_TIME));
        if (s.is_boosting) {
            s.boosting_time += dt; s.time_since_boosted = 0;
            s.boost_amount -= car::boost::USED_PER_SECOND * dt;
            float boost_accel = s.is_on_ground ? car::boost::ACCEL_GROUND : car::boost::ACCEL_AIR;
            Vec3 boost_force = s.rot.x_axis * physics::to_phys(boost_accel) * car::MASS;
            rb.apply_central_force(boost_force);
        } else { s.boosting_time = 0; s.time_since_boosted += dt; }
        s.boost_amount = fmaxf(0.0f, fminf(car::boost::MAX, s.boost_amount));
    }

    RS_INLINE bool ray_plane_intersect(const Vec3& from, const Vec3& to, const Vec3& plane_pos, const Vec3& plane_normal, Vec3& hit_point, Vec3& hit_normal) {
        Vec3 delta = to - from;
        float dist = delta.length();
        if (dist < SIMD_EPSILON) return false;
        Vec3 ray_direction = delta / dist;
        float dir_align = plane_normal.dot(ray_direction);
        if (fabsf(dir_align) < SIMD_EPSILON) return false;

        float normal_start = plane_normal.dot(from - plane_pos);
        float t = -normal_start / dir_align;
        if (t < 0.0f || t > dist) return false;

        hit_point = from + ray_direction * t;
        hit_normal = plane_normal;
        return true;
    }

    template<typename MeshType>
    RS_INLINE void update_vehicle_first(float dt, RigidBody* bodies, int ground_body_idx, MeshType* arena_mesh) {
        RigidBody& rb = get_body(bodies);
        RigidBody& ground = bodies[ground_body_idx];
        float friction_scale = car::MASS / 3.0f;

        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].update_transform(rb.collision.transform);
        }

        Vec3 sources[4], targets[4];
        float suspension_travels[4];

        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].prepare_raycast(rb.collision.transform, sources[i], targets[i], suspension_travels[i]);
        }

        constexpr int NUM_PLANES = 4;
        Vec3 plane_pos[NUM_PLANES] = {
            Vec3(0, 0, 0),
            Vec3(0, 0, arena::HEIGHT * physics::SCALE),
            Vec3(-arena::EXTENT_X * physics::SCALE, 0, 0),
            Vec3(arena::EXTENT_X * physics::SCALE, 0, 0)
        };
        Vec3 plane_nrm[NUM_PLANES] = {
            Vec3(0, 0, 1),
            Vec3(0, 0, -1),
            Vec3(1, 0, 0),
            Vec3(-1, 0, 0)
        };

        for (int i = 0; i < vehicle.num_wheels; i++) {
            Vec3 hit_normal, hit_point;
            bool hit = false;

            if (arena_mesh) {
                hit = arena_mesh->raycast(sources[i], targets[i], hit_normal, hit_point);
            }

            for (int p = 0; p < NUM_PLANES; p++) {
                Vec3 pl_hit_point, pl_hit_normal;
                if (ray_plane_intersect(sources[i], targets[i], plane_pos[p], plane_nrm[p], pl_hit_point, pl_hit_normal)) {
                    float mesh_dist = hit ? (hit_point - sources[i]).length_squared() : 1e18f;
                    float pl_dist = (pl_hit_point - sources[i]).length_squared();
                    if (pl_dist < mesh_dist) {
                        hit = true;
                        hit_point = pl_hit_point;
                        hit_normal = pl_hit_normal;
                    }
                }
            }

            if (hit) {
                vehicle.wheels[i].apply_raycast(rb, ground, ground_body_idx, true, hit_point, hit_normal, suspension_travels[i], dt);
            } else {
                vehicle.wheels[i].reset_suspension(suspension_travels[i]);
            }
        }

        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].calc_friction_impulses(rb, ground, friction_scale, dt);
        }
    }

    RS_INLINE void update_vehicle_second(float dt, RigidBody* bodies) {
        RigidBody& rb = get_body(bodies);
        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].update_suspension(rb, dt);
        }
        for (int i = 0; i < vehicle.num_wheels; i++) {
            vehicle.wheels[i].apply_friction(rb, dt);
        }
    }

    RS_INLINE void update_supersonic(float dt) {
        CarState& s = state;
        float speed_sq = s.vel.length_squared();
        float thresh = (s.is_supersonic && s.supersonic_time < car::supersonic::MAINTAIN_MAX_TIME) ?
            car::supersonic::MAINTAIN_MIN_SPEED * car::supersonic::MAINTAIN_MIN_SPEED :
            car::supersonic::START_SPEED * car::supersonic::START_SPEED;
        s.is_supersonic = speed_sq >= thresh;
        s.supersonic_time = s.is_supersonic ? s.supersonic_time + dt : 0;
    }

    RS_INLINE void finish_physics_tick(RigidBody& world_body) {
        if (state.is_demoed) return;

        if (velocity_impulse_cache.length_squared() > 0) {
            world_body.linear_vel = world_body.linear_vel + velocity_impulse_cache;
            velocity_impulse_cache = Vec3::zero();
        }

        float max_speed_phys = physics::to_phys(car::MAX_SPEED);
        if (world_body.linear_vel.length_squared() > max_speed_phys * max_speed_phys) {
            world_body.linear_vel = world_body.linear_vel.normalized() * max_speed_phys;
        }

        if (world_body.angular_vel.length_squared() > car::MAX_ANG_SPEED * car::MAX_ANG_SPEED) {
            world_body.angular_vel = world_body.angular_vel.normalized() * car::MAX_ANG_SPEED;
        }

        state.pos = physics::from_phys(world_body.collision.transform.translation);
        state.vel = physics::from_phys(world_body.linear_vel);
        state.ang_vel = world_body.angular_vel;
    }

};
