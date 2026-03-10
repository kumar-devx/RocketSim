#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"
#include "CudaCompat.h"

RS_NS_START

// Forward declarations
struct GpuArenaCollisionData;

/*
 * GPU Physics Pipeline Header
 * 
 * Defines the complete GPU-accelerated physics simulation pipeline.
 */

// Arena collision grid for efficient spatial queries
struct GpuArenaCollisionGrid {
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
    float cellSize;
    int gridDimX, gridDimY, gridDimZ;
    
    // In a full implementation, this would contain:
    // - Pointer to triangle mesh data
    // - BVH tree for collision queries
    // - Cell occupancy data for broad phase
};

// Apply one tick's worth of queued controls into the cars[] array on GPU.
// d_controls[i] maps to cars[d_controls[i].carArrayIdx].
// Call this each tick before LaunchGpuFullPhysicsStep in the multi-tick loop.
void LaunchApplyControlsKernel(
    GpuCarState* cars,
    int numCars,
    const GpuCarControls* d_controls,
    cudaStream_t stream
);

// Launch the complete GPU physics step
// Handles ball physics, car physics, and all collisions including mesh collisions
void LaunchGpuFullPhysicsStep(
    GpuBallState* balls,
    int numBalls,
    GpuCarState* cars,
    int numCars,
    float deltaTime,
    const GpuArenaCollisionGrid* collisionGrid,
    const GpuArenaCollisionData* arenaCollision,  // Phase 2: Mesh collision data
    cudaStream_t stream
);

RS_NS_END

#endif // RS_CUDA_ENABLED
