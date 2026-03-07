#pragma once

#ifdef RS_CUDA_ENABLED

#include "../../BaseInc.h"
#include <cuda_runtime.h>
#include <stdexcept>

RS_NS_START

// CUDA helper macros - FIXED: No longer calls exit(), throws exception instead
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            char error_msg[512]; \
            snprintf(error_msg, sizeof(error_msg), \
                "CUDA error in %s:%d: %s (%s)", \
                __FILE__, __LINE__, \
                cudaGetErrorString(err), #call); \
            fprintf(stderr, "%s\n", error_msg); \
            throw std::runtime_error(error_msg); \
        } \
    } while(0)

// Safer version that returns false on failure (for optional GPU usage)
#define CUDA_TRY(call) \
    ([&]() -> bool { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            fprintf(stderr, "CUDA warning in %s:%d: %s (%s)\n", \
                __FILE__, __LINE__, \
                cudaGetErrorString(err), #call); \
            return false; \
        } \
        return true; \
    })()

#define CUDA_KERNEL __global__
#define CUDA_DEVICE __device__
#define CUDA_HOST __host__
#define CUDA_HOST_DEVICE __host__ __device__

// GPU-friendly vector types (no constructors, POD)
struct GpuVec3 {
    float x, y, z;
    
    CUDA_HOST_DEVICE inline GpuVec3 operator+(const GpuVec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    
    CUDA_HOST_DEVICE inline GpuVec3 operator-(const GpuVec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    CUDA_HOST_DEVICE inline GpuVec3 operator*(float s) const {
        return {x * s, y * s, z * s};
    }
    
    CUDA_HOST_DEVICE inline float dot(const GpuVec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    CUDA_HOST_DEVICE inline float lengthSq() const {
        return x * x + y * y + z * z;
    }
    
    CUDA_HOST_DEVICE inline float length() const {
        return sqrtf(lengthSq());
    }
    
    CUDA_HOST_DEVICE inline GpuVec3 normalized() const {
        float len = length();
        return (len > 0) ? (*this * (1.0f / len)) : GpuVec3{0, 0, 0};
    }
};

struct GpuMat3x3 {
    float m[3][3];
    
    CUDA_HOST_DEVICE inline GpuVec3 operator*(const GpuVec3& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }
};

// GPU-compatible Ball state
struct GpuBallState {
    GpuVec3 pos;
    GpuVec3 vel;
    GpuVec3 angVel;
    GpuMat3x3 rotMat;
    
    float radius;
    float mass;
    float drag;
    float friction;
    float restitution;
    float maxSpeed;
    
    uint64_t tickCount;
};

// GPU-compatible Car state
struct GpuCarState {
    GpuVec3 pos;
    GpuVec3 vel;
    GpuVec3 angVel;
    GpuMat3x3 rotMat;
    
    // Wheel info (4 wheels max)
    struct WheelData {
        GpuVec3 connectionPoint;
        float suspensionLength;
        bool hasContact;
        GpuVec3 contactNormal;
        GpuVec3 contactPoint;
        float steerAngle;
    } wheels[4];
    
    int numWheels; // 3 or 4
    
    // Controls
    float throttle;
    float steer;
    float pitch;
    float yaw;
    float roll;
    bool jump;
    bool boost;
    bool handbrake;
    
    // State flags
    bool isOnGround;
    bool isDemoed;
    bool hasJumped;
    bool hasDoubleJumped;
    bool hasFlipped;
    bool isJumping;
    bool isFlipping;
    bool isBoosting;
    
    // Timers
    float jumpTime;
    float flipTime;
    float airTimeSinceJump;
    float boostingTime;
    float handbrakeVal;
    
    // Flip state
    GpuVec3 flipRelTorque;
    
    // Resources
    float boost_amount;
    float mass;
    
    // Configuration (copied from CarConfig)
    GpuVec3 hitboxSize;
    GpuVec3 hitboxPosOffset;
    
    uint64_t tickCount;
};

// Arena configuration for GPU
struct GpuArenaConfig {
    float gravity;
    float tickTime;
    int numCars;
    bool hasBall;
};

// Conversion helpers (CPU <-> GPU)
inline GpuVec3 ToGpuVec3(const Vec& v) {
    return {v.x, v.y, v.z};
}

inline Vec FromGpuVec3(const GpuVec3& v) {
    return Vec(v.x, v.y, v.z);
}

#ifndef __CUDACC__  // Bullet conversions only in C++ (not CUDA)
inline GpuMat3x3 ToGpuMat3x3(const btMatrix3x3& m) {
    GpuMat3x3 result;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result.m[i][j] = m[i][j];
    return result;
}

inline btMatrix3x3 FromGpuMat3x3(const GpuMat3x3& m) {
    btMatrix3x3 result;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result[i][j] = m.m[i][j];
    return result;
}
#else  // CUDA versions using RotMat
inline GpuMat3x3 ToGpuMat3x3(const RotMat& m) {
    GpuMat3x3 result;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result.m[i][j] = m[i][j];
    return result;
}

inline RotMat FromGpuMat3x3(const GpuMat3x3& m) {
    return RotMat(
        Vec(m.m[0][0], m.m[1][0], m.m[2][0]),
        Vec(m.m[0][1], m.m[1][1], m.m[2][1]),
        Vec(m.m[0][2], m.m[1][2], m.m[2][2])
    );
}
#endif

RS_NS_END

#endif // RS_CUDA_ENABLED
