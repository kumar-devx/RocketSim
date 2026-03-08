#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// GPU constants from RLConst (device-side)
// Declared here, defined in GpuConstants.cu to avoid multiple definitions
namespace GpuRLConst {
    // Physics constants
    extern __device__ __constant__ float GRAVITY_Z;
    extern __device__ __constant__ float CAR_MASS;
    extern __device__ __constant__ float BALL_MASS;
    
    // Speed limits
    extern __device__ __constant__ float CAR_MAX_SPEED;
    extern __device__ __constant__ float BALL_MAX_SPEED;
    extern __device__ __constant__ float CAR_MAX_ANG_SPEED;
    extern __device__ __constant__ float BALL_MAX_ANG_SPEED;
    
    // Boost constants
    extern __device__ __constant__ float BOOST_MAX;
    extern __device__ __constant__ float BOOST_USED_PER_SECOND;
    extern __device__ __constant__ float BOOST_MIN_TIME;
    extern __device__ __constant__ float BOOST_ACCEL_GROUND;
    extern __device__ __constant__ float BOOST_ACCEL_AIR;
    
    // Jump constants
    extern __device__ __constant__ float JUMP_ACCEL;
    extern __device__ __constant__ float JUMP_IMMEDIATE_FORCE;
    extern __device__ __constant__ float JUMP_MIN_TIME;
    extern __device__ __constant__ float JUMP_MAX_TIME;
    extern __device__ __constant__ float DOUBLEJUMP_MAX_DELAY;
    
    // Flip constants
    extern __device__ __constant__ float FLIP_TORQUE_TIME;
    extern __device__ __constant__ float FLIP_Z_DAMPING_START;
    extern __device__ __constant__ float FLIP_Z_DAMPING_END;
    extern __device__ __constant__ float FLIP_TORQUE_X;
    extern __device__ __constant__ float FLIP_TORQUE_Y;
    extern __device__ __constant__ float FLIP_INITIAL_VEL_SCALE;
    
    // Air control
    extern __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_X;
    extern __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_Y;
    extern __device__ __constant__ float CAR_AIR_CONTROL_TORQUE_Z;
    extern __device__ __constant__ float CAR_AIR_CONTROL_DAMPING;
    
    // Throttle/brake
    extern __device__ __constant__ float THROTTLE_TORQUE_AMOUNT;
    extern __device__ __constant__ float BRAKE_TORQUE_AMOUNT;
    extern __device__ __constant__ float THROTTLE_AIR_ACCEL;
    extern __device__ __constant__ float STOPPING_FORWARD_VEL;
    extern __device__ __constant__ float COASTING_BRAKE_FACTOR;
    
    // Suspension
    extern __device__ __constant__ float SUSPENSION_STIFFNESS;
    extern __device__ __constant__ float WHEELS_DAMPING_COMPRESSION;
    extern __device__ __constant__ float WHEELS_DAMPING_RELAXATION;
    extern __device__ __constant__ float MAX_SUSPENSION_TRAVEL;
    
    // Collision
    extern __device__ __constant__ float BALL_FRICTION;
    extern __device__ __constant__ float BALL_RESTITUTION;
    extern __device__ __constant__ float CAR_COLLISION_FRICTION;
    extern __device__ __constant__ float CAR_COLLISION_RESTITUTION;
    extern __device__ __constant__ float CARBALL_COLLISION_FRICTION;
    extern __device__ __constant__ float CARBALL_COLLISION_RESTITUTION;
}

RS_NS_END

#endif // RS_CUDA_ENABLED
