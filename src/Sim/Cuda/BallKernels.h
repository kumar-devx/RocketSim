#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// Forward declarations
void LaunchBallPhysicsKernel(GpuBallState* balls, int numBalls, float deltaTime, cudaStream_t stream = 0);

void LaunchBallIntegrationKernel(GpuBallState* balls, int numBalls, float deltaTime, cudaStream_t stream = 0);

RS_NS_END

#endif // RS_CUDA_ENABLED
