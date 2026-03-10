#pragma once

#include "../math/base.cuh"
#include "../math/vec.cuh"

namespace physics {
    constexpr float SCALE = 0.02f;
    constexpr float INV_SCALE = 50.0f;
    constexpr float TICK_RATE = 120.0f;
    constexpr float TICK_TIME = 1.0f / TICK_RATE;

    RS_INLINE Vec3 to_phys(const Vec3& v) { return v * SCALE; }
    RS_INLINE float to_phys(float v) { return v * SCALE; }
    RS_INLINE Vec3 from_phys(const Vec3& v) { return v * INV_SCALE; }
    RS_INLINE float from_phys(float v) { return v * INV_SCALE; }
}
constexpr float GRAVITY_Z = -650.0f;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float FRAC_PI_2 = PI_F / 2.0f;
constexpr float FRAC_PI_4 = PI_F / 4.0f;
constexpr float FRAC_1_SQRT_2 = 0.7071067811865476f;

namespace arena {
    constexpr float EXTENT_X = 4096.0f;
    constexpr float EXTENT_Y = 5120.0f;
    constexpr float HEIGHT = 2048.0f;
    constexpr float FRICTION = 0.6f;
    constexpr float RESTITUTION = 0.3f;
}

namespace car {
    constexpr float MASS = 180.0f;
    constexpr float FRICTION = 0.3f;
    constexpr float RESTITUTION = 0.1f;
    constexpr float HIT_BALL_FRICTION = 2.0f;
    constexpr float HIT_BALL_RESTITUTION = 0.0f;
    constexpr float HIT_WORLD_FRICTION = 0.3f;
    constexpr float HIT_WORLD_RESTITUTION = 0.3f;
    constexpr float HIT_CAR_FRICTION = 0.09f;
    constexpr float HIT_CAR_RESTITUTION = 0.1f;
    constexpr float MAX_SPEED = 2300.0f;
    constexpr float MAX_ANG_SPEED = 5.5f;

    namespace boost {
        constexpr float MAX = 100.0f;
        constexpr float USED_PER_SECOND = MAX / 3.0f;
        constexpr float MIN_TIME = 0.1f;
        constexpr float ACCEL_GROUND = 2975.0f / 3.0f;
        constexpr float ACCEL_AIR = 3175.0f / 3.0f;
        constexpr float SPAWN_AMOUNT = MAX / 3.0f;
        constexpr float RECHARGE_PER_SECOND = 10.0f;
        constexpr float RECHARGE_DELAY = 0.25f;
    }

    namespace supersonic {
        constexpr float START_SPEED = 2200.0f;
        constexpr float MAINTAIN_MIN_SPEED = START_SPEED - 100.0f;
        constexpr float MAINTAIN_MAX_TIME = 1.0f;
    }

    namespace drive {
        constexpr float THROTTLE_TORQUE = MASS * 400.0f;
        constexpr float BRAKE_TORQUE = MASS * (14.25f + (1.0f / 3.0f));
        constexpr float STOPPING_FORWARD_VEL = 25.0f;
        constexpr float COASTING_BRAKE_FACTOR = 0.15f;
        constexpr float BRAKING_NO_THROTTLE_THRESH = 0.01f;
        constexpr float THROTTLE_DEADZONE = 0.001f;
        constexpr float THROTTLE_AIR_ACCEL = 200.0f / 3.0f;
        constexpr float POWERSLIDE_RISE_RATE = 5.0f;
        constexpr float POWERSLIDE_FALL_RATE = 2.0f;
    }

    namespace jump {
        constexpr float ACCEL = 4375.0f / 3.0f;
        constexpr float IMMEDIATE_FORCE = 875.0f / 3.0f;
        constexpr float MIN_TIME = 0.025f;
        constexpr float RESET_TIME_PAD = 1.0f / 40.0f;
        constexpr float MAX_TIME = 0.2f;
        constexpr float DOUBLEJUMP_MAX_DELAY = 1.25f;
    }

    namespace flip {
        constexpr float Z_DAMP_120 = 0.35f;
        constexpr float Z_DAMP_START = 0.15f;
        constexpr float Z_DAMP_END = 0.21f;
        constexpr float TORQUE_TIME = 0.65f;
        constexpr float TORQUE_MIN_TIME = 0.41f;
        constexpr float PITCHLOCK_TIME = 1.0f;
        constexpr float PITCHLOCK_EXTRA_TIME = 0.3f;
        constexpr float INITIAL_VEL_SCALE = 500.0f;
        constexpr float TORQUE_X = 260.0f;
        constexpr float TORQUE_Y = 224.0f;
        constexpr float FORWARD_IMPULSE_MAX_SPEED_SCALE = 1.0f;
        constexpr float SIDE_IMPULSE_MAX_SPEED_SCALE = 1.9f;
        constexpr float BACKWARD_IMPULSE_MAX_SPEED_SCALE = 2.5f;
        constexpr float BACKWARD_IMPULSE_SCALE_X = 16.0f / 15.0f;
    }

    namespace air_control {
        constexpr float TORQUE_X = 130.0f;
        constexpr float TORQUE_Y = 95.0f;
        constexpr float TORQUE_Z = 400.0f;
        constexpr float DAMPING_X = 30.0f;
        constexpr float DAMPING_Y = 20.0f;
        constexpr float DAMPING_Z = 50.0f;
        constexpr float TORQUE_APPLY_SCALE = 2.0f * PI_F / 65536.0f * 1000.0f;
    }

    namespace autoflip {
        constexpr float IMPULSE = 200.0f;
        constexpr float TORQUE = 50.0f;
        constexpr float TIME = 0.4f;
        constexpr float NORM_Z_THRESH = FRAC_1_SQRT_2;
        constexpr float ROLL_THRESH = 2.8f;
    }

    namespace autoroll {
        constexpr float FORCE = 100.0f;
        constexpr float TORQUE = 80.0f;
    }

    namespace bump {
        constexpr float COOLDOWN_TIME = 0.25f;
        constexpr float MIN_FORWARD_DIST = 64.5f;
    }

    namespace spawn {
        constexpr float REST_Z = 17.0f;
        constexpr float SPAWN_Z = 36.0f;
        constexpr float RESPAWN_TIME = 3.0f;
        constexpr float RESPAWN_YAW = 1.5707963f;
        constexpr int NUM_RESPAWN_LOCS = 4;
        RS_INLINE Vec3 get_respawn_pos(int idx, int team) {
            float x, y;
            switch (idx & 3) {
                case 0: x = -2304.f; y = -4608.f; break;
                case 1: x = -2688.f; y = -4608.f; break;
                case 2: x =  2304.f; y = -4608.f; break;
                default: x =  2688.f; y = -4608.f; break;
            }
            float y_dir = (float)(team * 2 - 1);
            return Vec3(x, y * -y_dir, SPAWN_Z);
        }
    }
}

namespace ball {
    constexpr float RADIUS = 91.25f;
    constexpr float MASS = 30.0f;
    constexpr float REST_Z = 93.15f;
    constexpr float MAX_ANG_SPEED = 6.0f;
    constexpr float DRAG = 0.03f;
    constexpr float FRICTION = 0.35f;
    constexpr float RESTITUTION = 0.6f;
    constexpr float MAX_SPEED = 6000.0f;

    namespace car_hit {
        constexpr float FORWARD_SCALE = 0.65f;
        constexpr float MAX_DELTA_VEL = 4600.0f;
        constexpr float Z_SCALE_NORMAL = 0.35f;
    }
}

namespace vehicle {
    constexpr float SUSPENSION_FORCE_FRONT = 36.0f - 0.25f;
    constexpr float SUSPENSION_FORCE_BACK = 54.0f + 0.25f + 0.015f;
    constexpr float SUSPENSION_STIFFNESS = 500.0f;
    constexpr float DAMPING_COMPRESSION = 25.0f;
    constexpr float DAMPING_RELAXATION = 40.0f;
    constexpr float MAX_SUSPENSION_TRAVEL = 12.0f;
    constexpr float SUSPENSION_SUBTRACTION = 0.05f;
}

namespace goal {
    constexpr float SCORE_THRESHOLD_Y = 5124.25f;
}

struct LinearPieceCurve6 {
    float x[6], y[6];
    int n;

    RS_INLINE float eval(float v) const {
        if (v <= x[0]) return y[0];
        for (int i = 1; i < n; i++) {
            if (v <= x[i]) {
                float t = (v - x[i-1]) / (x[i] - x[i-1]);
                return y[i-1] + t * (y[i] - y[i-1]);
            }
        }
        return y[n-1];
    }
};

namespace curves {
    constexpr float HANDBRAKE_LAT_FRICTION_FACTOR = 0.9f;

    RS_INLINE LinearPieceCurve6 make_curve(float x0, float x1, float x2, float x3, float x4, float x5,
                                            float y0, float y1, float y2, float y3, float y4, float y5, int n) {
        LinearPieceCurve6 c;
        c.x[0] = x0; c.x[1] = x1; c.x[2] = x2; c.x[3] = x3; c.x[4] = x4; c.x[5] = x5;
        c.y[0] = y0; c.y[1] = y1; c.y[2] = y2; c.y[3] = y3; c.y[4] = y4; c.y[5] = y5;
        c.n = n;
        return c;
    }

    RS_INLINE float eval_steer_angle(float v) {
        if (v <= 0) return 0.53356f;
        if (v <= 500) return 0.53356f + (v - 0) / 500.0f * (0.31930f - 0.53356f);
        if (v <= 1000) return 0.31930f + (v - 500) / 500.0f * (0.18203f - 0.31930f);
        if (v <= 1500) return 0.18203f + (v - 1000) / 500.0f * (0.10570f - 0.18203f);
        if (v <= 1750) return 0.10570f + (v - 1500) / 250.0f * (0.08507f - 0.10570f);
        if (v <= 3000) return 0.08507f + (v - 1750) / 1250.0f * (0.03454f - 0.08507f);
        return 0.03454f;
    }

    RS_INLINE float eval_powerslide_steer(float v) {
        if (v <= 0) return 0.39235f;
        if (v <= 2500) return 0.39235f + (v - 0) / 2500.0f * (0.12610f - 0.39235f);
        return 0.12610f;
    }

    RS_INLINE float eval_drive_torque_factor(float v) {
        if (v <= 0) return 1.0f;
        if (v <= 1400) return 1.0f + (v - 0) / 1400.0f * (0.1f - 1.0f);
        if (v <= 1410) return 0.1f + (v - 1400) / 10.0f * (0.0f - 0.1f);
        return 0.0f;
    }

    RS_INLINE float eval_non_sticky_friction(float v) {
        if (v <= 0) return 0.1f;
        if (v <= 0.7075f) return 0.1f + (v - 0) / 0.7075f * (0.5f - 0.1f);
        if (v <= 1.0f) return 0.5f + (v - 0.7075f) / (1.0f - 0.7075f) * (1.0f - 0.5f);
        return 1.0f;
    }

    RS_INLINE float eval_lat_friction(float v) {
        if (v <= 0) return 1.0f;
        if (v <= 1) return 1.0f + v * (0.2f - 1.0f);
        return 0.2f;
    }

    RS_INLINE float eval_handbrake_long_friction(float v) {
        if (v <= 0) return 0.5f;
        if (v <= 1) return 0.5f + v * (0.9f - 0.5f);
        return 0.9f;
    }

    RS_INLINE float eval_ball_car_extra_impulse(float v) {
        if (v <= 0) return 0.65f;
        if (v <= 500) return 0.65f;
        if (v <= 2300) return 0.65f + (v - 500) / 1800.0f * (0.55f - 0.65f);
        if (v <= 4600) return 0.55f + (v - 2300) / 2300.0f * (0.30f - 0.55f);
        return 0.30f;
    }

    RS_INLINE float eval_bump_vel_ground(float v) {
        if (v <= 0) return 5.0f / 6.0f;
        if (v <= 1400) return 5.0f / 6.0f + (v - 0) / 1400.0f * (1100.0f - 5.0f / 6.0f);
        if (v <= 2200) return 1100.0f + (v - 1400) / 800.0f * (1530.0f - 1100.0f);
        return 1530.0f;
    }

    RS_INLINE float eval_bump_vel_air(float v) {
        if (v <= 0) return 5.0f / 6.0f;
        if (v <= 1400) return 5.0f / 6.0f + (v - 0) / 1400.0f * (1390.0f - 5.0f / 6.0f);
        if (v <= 2200) return 1390.0f + (v - 1400) / 800.0f * (1945.0f - 1390.0f);
        return 1945.0f;
    }

    RS_INLINE float eval_bump_up_vel(float v) {
        if (v <= 0) return 2.0f / 6.0f;
        if (v <= 1400) return 2.0f / 6.0f + (v - 0) / 1400.0f * (278.0f - 2.0f / 6.0f);
        if (v <= 2200) return 278.0f + (v - 1400) / 800.0f * (417.0f - 278.0f);
        return 417.0f;
    }

}
