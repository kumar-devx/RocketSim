#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"

RS_NS_START

// Forward declarations
void LaunchCarPhysicsKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream = 0);

void LaunchCarIntegrationKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream = 0);

// GPU Car-to-Car Collision Detection & Physics (Phase 2-3 Optimization)
// Detects and resolves collisions between all cars directly on GPU
// Applies impulses directly to car velocities (no CPU callback needed)
// Expected speedup: 2.5-3x (vs 2x current without this optimization)
// CRITICAL: Must be called before _SyncStatesFromGPU() to get updated velocities
void LaunchCarToCarCollisionFull(
    GpuCarState* cars,
    int numCars,
    float carMass,
    cudaStream_t stream = 0
);

RS_NS_END

#endif // RS_CUDA_ENABLED
