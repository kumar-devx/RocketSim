#pragma once

#include "GpuTypes.h"
#include "CudaCompat.h"
#include <vector>

RS_NS_START

/*
 * GPU Mesh Collision System - Phase 2 Implementation
 * 
 * Provides efficient arena mesh collision detection on GPU using:
 * - Bounding Volume Hierarchy (BVH) for fast spatial queries
 * - Triangle mesh data transferred to unified memory
 * - Sphere-triangle collision detection
 * - Support for multiple collision meshes (arena geometry)
 */

// Triangle representation for GPU
struct GpuTriangle {
    GpuVec3 v0, v1, v2;
    GpuVec3 normal;  // Pre-computed normal
    float padding;
};

// AABB node in BVH tree
struct GpuBvhNode {
    GpuVec3 aabbMin;
    int childLeft;      // -1 if leaf, otherwise index to left child
    
    GpuVec3 aabbMax;
    int childRight;     // -1 if leaf, otherwise index to right child
    
    int triStart;       // First triangle index (for leaf nodes)
    int triCount;       // Number of triangles (for leaf nodes)
    int padding[2];
};

// GPU-side mesh with BVH
struct GpuMesh {
    GpuTriangle* triangles;
    int numTriangles;
    
    GpuBvhNode* bvhNodes;
    int numBvhNodes;
    
    GpuVec3 aabbMin;
    GpuVec3 aabbMax;
};

// Collection of meshes for an arena
struct GpuArenaCollisionData {
    GpuTriangle* triangles;
    int triangleCount;
    
    GpuBvhNode* bvhNodes;
    int bvhNodeCount;
    
    GpuMesh* meshes;
    int numMeshes;
};

// CPU-side BVH builder
class CpuBvhBuilder {
public:
    struct Triangle {
        GpuVec3 v0, v1, v2;
        GpuVec3 normal;
        float padding;
    };
    
    struct BvhNode {
        GpuVec3 aabbMin;
        int childLeft;
        GpuVec3 aabbMax;
        int childRight;
        int triStart;
        int triCount;
        int padding[2];
    };
    
    // Build BVH from triangle list
    static std::vector<BvhNode> BuildBvh(const std::vector<Triangle>& triangles, int maxTrisPerLeaf = 4);
    
private:
    struct BuildNode {
        GpuVec3 aabbMin, aabbMax;
        int triStart, triCount;
        int childLeft, childRight;
    };
    
    static int BuildRecursive(
        const std::vector<Triangle>& triangles,
        std::vector<BuildNode>& buildNodes,
        int nodeIdx,
        int depth,
        int maxTrisPerLeaf
    );
};

// Load collision meshes to GPU
void LoadArenaCollisionMeshesToGpu(
    const std::vector<struct CollisionMeshFile>& meshFiles,
    GpuArenaCollisionData* outData,
    cudaStream_t stream
);

// Load from raw triangle data (used by Arena with Bullet shapes)
void LoadArenaCollisionMeshesFromTriangles(
    const std::vector<CpuBvhBuilder::Triangle>& triangles,
    GpuArenaCollisionData* outData,
    cudaStream_t stream
);

// Cleanup GPU mesh data
void UnloadArenaCollisionMeshes(GpuArenaCollisionData* data);

// Query GPU collision kernels
void LaunchMeshCollisionKernel(
    GpuBallState* balls,
    int numBalls,
    const GpuArenaCollisionData* arenaCollision,
    float dt,
    cudaStream_t stream
);

// GPU wheel raycasting kernel
void LaunchWheelRaycastKernel(
    GpuCarState* cars,
    int numCars,
    const GpuArenaCollisionData* arenaCollision,
    float wheelRadius,
    cudaStream_t stream
);

// GPU suspension force application kernel
void LaunchApplySuspensionForcesKernel(
    GpuCarState* cars,
    int numCars,
    float deltaTime,
    cudaStream_t stream
);

RS_NS_END
