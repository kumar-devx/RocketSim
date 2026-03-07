#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// Forward declarations
void LaunchCarPhysicsKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream = 0);

void LaunchCarIntegrationKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream = 0);

RS_NS_END

#endif // RS_CUDA_ENABLED
