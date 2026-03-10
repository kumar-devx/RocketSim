#pragma once

#include "constants.cuh"

namespace boost_pads {
    constexpr float CYL_HEIGHT = 95.0f;
    constexpr float CYL_RAD_BIG = 208.0f;
    constexpr float CYL_RAD_SMALL = 144.0f;
    constexpr float BOX_HEIGHT = 64.0f;
    constexpr float BOX_RAD_BIG = 160.0f;
    constexpr float BOX_RAD_SMALL = 120.0f;
    constexpr float COOLDOWN_BIG = 10.0f;
    constexpr float COOLDOWN_SMALL = 4.0f;
    constexpr float AMOUNT_BIG = 100.0f;
    constexpr float AMOUNT_SMALL = 12.0f;
}

struct BoostPadConfig {
    Vec3 pos;
    bool is_big;
};

struct BoostPadState {
    bool is_active;
    float cooldown_timer;

    RS_INLINE BoostPadState() : is_active(true), cooldown_timer(0) {}
};

struct BoostPad {
    BoostPadConfig config;
    BoostPadState state;
    float radius;

    RS_INLINE BoostPad() : radius(0) {}

    RS_INLINE void init(const Vec3& p, bool big) {
        config.pos = p;
        config.is_big = big;
        radius = big ? boost_pads::CYL_RAD_BIG : boost_pads::CYL_RAD_SMALL;
        state.is_active = true;
        state.cooldown_timer = 0;
    }

    RS_INLINE float get_cooldown() const {
        return config.is_big ? boost_pads::COOLDOWN_BIG : boost_pads::COOLDOWN_SMALL;
    }

    RS_INLINE float get_boost_amount() const {
        return config.is_big ? boost_pads::AMOUNT_BIG : boost_pads::AMOUNT_SMALL;
    }

    RS_INLINE void update(float dt) {
        if (!state.is_active) {
            state.cooldown_timer -= dt;
            if (state.cooldown_timer <= 0) {
                state.is_active = true;
                state.cooldown_timer = 0;
            }
        }
    }

    RS_INLINE bool try_pickup(const Vec3& car_pos, float& car_boost) {
        if (!state.is_active) return false;

        float z_diff = fabsf(car_pos.z - config.pos.z);
        if (z_diff > boost_pads::CYL_HEIGHT) return false;

        Vec3 diff = car_pos - config.pos;
        diff.z = 0;
        float dist_sq = diff.length_squared();

        if (dist_sq <= radius * radius) {
            float amount = get_boost_amount();
            car_boost = fminf(car::boost::MAX, car_boost + amount);
            state.is_active = false;
            state.cooldown_timer = get_cooldown();
            return true;
        }
        return false;
    }
};

constexpr int NUM_BIG_PADS = 6;
constexpr int NUM_SMALL_PADS = 28;
constexpr int TOTAL_PADS = NUM_BIG_PADS + NUM_SMALL_PADS;

RS_INLINE Vec3 get_big_pad_loc(int i) {
    switch (i) {
        case 0: return Vec3(-3584.0f, 0.0f, 73.0f);
        case 1: return Vec3(3584.0f, 0.0f, 73.0f);
        case 2: return Vec3(-3072.0f, 4096.0f, 73.0f);
        case 3: return Vec3(3072.0f, 4096.0f, 73.0f);
        case 4: return Vec3(-3072.0f, -4096.0f, 73.0f);
        default: return Vec3(3072.0f, -4096.0f, 73.0f);
    }
}

RS_INLINE Vec3 get_small_pad_loc(int i) {
    switch (i) {
        case 0: return Vec3(0.0f, -4240.0f, 70.0f);
        case 1: return Vec3(-1792.0f, -4184.0f, 70.0f);
        case 2: return Vec3(1792.0f, -4184.0f, 70.0f);
        case 3: return Vec3(-940.0f, -3308.0f, 70.0f);
        case 4: return Vec3(940.0f, -3308.0f, 70.0f);
        case 5: return Vec3(0.0f, -2816.0f, 70.0f);
        case 6: return Vec3(-3584.0f, -2484.0f, 70.0f);
        case 7: return Vec3(3584.0f, -2484.0f, 70.0f);
        case 8: return Vec3(-1788.0f, -2300.0f, 70.0f);
        case 9: return Vec3(1788.0f, -2300.0f, 70.0f);
        case 10: return Vec3(-2048.0f, -1036.0f, 70.0f);
        case 11: return Vec3(0.0f, -1024.0f, 70.0f);
        case 12: return Vec3(2048.0f, -1036.0f, 70.0f);
        case 13: return Vec3(-1024.0f, 0.0f, 70.0f);
        case 14: return Vec3(1024.0f, 0.0f, 70.0f);
        case 15: return Vec3(-2048.0f, 1036.0f, 70.0f);
        case 16: return Vec3(0.0f, 1024.0f, 70.0f);
        case 17: return Vec3(2048.0f, 1036.0f, 70.0f);
        case 18: return Vec3(-1788.0f, 2300.0f, 70.0f);
        case 19: return Vec3(1788.0f, 2300.0f, 70.0f);
        case 20: return Vec3(-3584.0f, 2484.0f, 70.0f);
        case 21: return Vec3(3584.0f, 2484.0f, 70.0f);
        case 22: return Vec3(0.0f, 2816.0f, 70.0f);
        case 23: return Vec3(-940.0f, 3308.0f, 70.0f);
        case 24: return Vec3(940.0f, 3308.0f, 70.0f);
        case 25: return Vec3(-1792.0f, 4184.0f, 70.0f);
        case 26: return Vec3(1792.0f, 4184.0f, 70.0f);
        default: return Vec3(0.0f, 4240.0f, 70.0f);
    }
}
