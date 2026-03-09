#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// GPU constants from RLConst (device-side)
// Defined in header with __device__ __constant__ to avoid multiple definitions
namespace GpuRLConst {
    // Physics constants
    __device__ __constant__ float GRAVITY_Z = -650.0f;
    __device__ __constant__ float CAR_MASS = 180.0f;
    __device__ __constant__ float BALL_MASS = 30.0f;
    
    // Speed limits
    __device__ __constant__ float CAR_MAX_SPEED = 2300.0f;
    __device__ __constant__ float BALL_MAX_SPEED = 6000.0f;
    __device__ __constant__ float CAR_MAX_ANG_SPEED = 5.5f;
    __device__ __constant__ float BALL_MAX_ANG_SPEED = 6.0f;
    
    // Boost constants
    __device__ __constant__ float BOOST_MAX = 100.0f;
    __device__ __constant__ float BOOST_USED_PER_SECOND = 33.333f;
    __device__ __constant__ float BOOST_MIN_TIME = 0.1f;
    __device__ __constant__ float BOOST_ACCEL_GROUND = 991.667f;
    __device__ __constant__ float BOOST_ACCEL_AIR = 1058.333f;
    
    // Jump constants
    __device__ __constant__ float JUMP_ACCEL = 1458.333f;
    __device__ __constant__ float JUMP_IMMEDIATE_FORCE = 291.667f;
    __device__ __constant__ float JUMP_MIN_TIME = 0.025f;
    __device__ __constant__ float JUMP_MAX_TIME = 0.2f;
    __device__ __constant__ float DOUBLEJUMP_MAX_DELAY = 1.25f;
    
    // Flip constants
    __device__ __constant__ float FLIP_TORQUE_TIME = 0.65f;
    __device__ __constant__ float FLIP_Z_DAMPING_START = 0.15f;
    __device__ __constant__ float FLIP_Z_DAMPING_END = 0.21f;
    __device__ __constant__ float FLIP_TORQUE_X = 260.0f;
    __device__ __constant__ float FLIP_TORQUE_Y = 224.0f;
    __device__ __constant__ float FLIP_INITIAL_VEL_SCALE = 0.5f;
    
    // Air control
    __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_X = 130.0f;
    __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_Y = 95.0f;
    __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_Z = 400.0f;
    __device__ __constant__ float CAR_AIR_CONTROL_DAMPING = 30.0f;
    
    // Throttle/brake
    __device__ __constant__ float THROTTLE_TORQUE_AMOUNT = 72000.0f;
    __device__ __constant__ float BRAKE_TORQUE_AMOUNT = 2565.0f;
    __device__ __constant__ float THROTTLE_AIR_ACCEL = 66.667f;
    __device__ __constant__ float STOPPING_FORWARD_VEL = 25.0f;
    __device__ __constant__ float COASTING_BRAKE_FACTOR = 0.15f;
    
    // Suspension
    __device__ __constant__ float SUSPENSION_STIFFNESS = 500.0f;
    __device__ __constant__ float WHEELS_DAMPING_COMPRESSION = 25.0f;
    __device__ __constant__ float WHEELS_DAMPING_RELAXATION = 40.0f;
    __device__ __constant__ float MAX_SUSPENSION_TRAVEL = 12.0f;
    
    // Collision
    __device__ __constant__ float BALL_FRICTION = 0.35f;
    __device__ __constant__ float BALL_RESTITUTION = 0.6f;
    __device__ __constant__ float CAR_COLLISION_FRICTION = 0.3f;
    __device__ __constant__ float CAR_COLLISION_RESTITUTION = 0.1f;
    __device__ __constant__ float CARBALL_COLLISION_FRICTION = 2.0f;
    __device__ __constant__ float CARBALL_COLLISION_RESTITUTION = 0.0f;
}

RS_NS_END

#endif // RS_CUDA_ENABLED
