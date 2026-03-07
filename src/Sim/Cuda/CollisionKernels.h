#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// Simplified collision detection for GPU
// Full mesh collision will use Bullet on CPU for now

// Ball-floor collision
void LaunchBallFloorCollisionKernel(GpuBallState* balls, int numBalls, cudaStream_t stream = 0);

// Ball-car collision (sphere vs box)
void LaunchBallCarCollisionKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int numBalls,
    int numCars,
    cudaStream_t stream = 0
);

// Ground detection for cars (simple height check)
void LaunchCarGroundDetectionKernel(GpuCarState* cars, int numCars, cudaStream_t stream = 0);

RS_NS_END

#endif // RS_CUDA_ENABLED
