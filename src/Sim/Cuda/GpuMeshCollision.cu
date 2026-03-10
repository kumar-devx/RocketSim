#ifdef RS_CUDA_ENABLED

#include "GpuMeshCollision.h"
#include "../../CollisionMeshFile/CollisionMeshFile.h"
#include <algorithm>
#include <iostream>
#include <cstring>

RS_NS_START

// ============================================================================
// CPU-SIDE BVH BUILDER
// ============================================================================

std::vector<CpuBvhBuilder::BvhNode> CpuBvhBuilder::BuildBvh(
    const std::vector<Triangle>& triangles,
    int maxTrisPerLeaf
) {
    if (triangles.empty()) {
        return {};
    }
    
    std::vector<BuildNode> buildNodes;
    buildNodes.reserve(triangles.size() + 100);  // Pre-allocate
    
    // Create root node
    BuildNode root;
    root.triStart = 0;
    root.triCount = triangles.size();
    root.childLeft = -1;
    root.childRight = -1;
    
    // Calculate root AABB
    root.aabbMin = triangles[0].v0;
    root.aabbMax = triangles[0].v0;
    
    for (const auto& tri : triangles) {
        root.aabbMin.x = fminf(root.aabbMin.x, fminf(tri.v0.x, fminf(tri.v1.x, tri.v2.x)));
        root.aabbMin.y = fminf(root.aabbMin.y, fminf(tri.v0.y, fminf(tri.v1.y, tri.v2.y)));
        root.aabbMin.z = fminf(root.aabbMin.z, fminf(tri.v0.z, fminf(tri.v1.z, tri.v2.z)));
        
        root.aabbMax.x = fmaxf(root.aabbMax.x, fmaxf(tri.v0.x, fmaxf(tri.v1.x, tri.v2.x)));
        root.aabbMax.y = fmaxf(root.aabbMax.y, fmaxf(tri.v0.y, fmaxf(tri.v1.y, tri.v2.y)));
        root.aabbMax.z = fmaxf(root.aabbMax.z, fmaxf(tri.v0.z, fmaxf(tri.v1.z, tri.v2.z)));
    }
    
    buildNodes.push_back(root);
    
    // Recursively build tree
    BuildRecursive(triangles, buildNodes, 0, 0, maxTrisPerLeaf);
    
    // Convert to final format
    std::vector<BvhNode> result;
    result.reserve(buildNodes.size());
    
    for (const auto& bn : buildNodes) {
        BvhNode node;
        node.aabbMin = bn.aabbMin;
        node.aabbMax = bn.aabbMax;
        node.childLeft = bn.childLeft;
        node.childRight = bn.childRight;
        node.triStart = bn.triStart;
        node.triCount = bn.triCount;
        node.padding[0] = 0;
        node.padding[1] = 0;
        result.push_back(node);
    }
    
    return result;
}

int CpuBvhBuilder::BuildRecursive(
    const std::vector<Triangle>& triangles,
    std::vector<BuildNode>& buildNodes,
    int nodeIdx,
    int depth,
    int maxTrisPerLeaf
) {
    BuildNode& node = buildNodes[nodeIdx];
    
    // Base case: leaf node
    if (node.triCount <= maxTrisPerLeaf) {
        node.childLeft = -1;
        node.childRight = -1;
        return nodeIdx;
    }
    
    // Find split axis (use longest AABB dimension)
    GpuVec3 extent;
    extent.x = node.aabbMax.x - node.aabbMin.x;
    extent.y = node.aabbMax.y - node.aabbMin.y;
    extent.z = node.aabbMax.z - node.aabbMin.z;
    
    int splitAxis = 0;
    if (extent.y > extent.x) splitAxis = 1;
    if (extent.z > (splitAxis == 0 ? extent.x : extent.y)) splitAxis = 2;
    
    // Get split position
    float splitPos = 0;
    if (splitAxis == 0) {
        splitPos = (node.aabbMin.x + node.aabbMax.x) * 0.5f;
    } else if (splitAxis == 1) {
        splitPos = (node.aabbMin.y + node.aabbMax.y) * 0.5f;
    } else {
        splitPos = (node.aabbMin.z + node.aabbMax.z) * 0.5f;
    }
    
    // Partition triangles
    int splitIdx = node.triStart;
    for (int i = node.triStart; i < node.triStart + node.triCount; i++) {
        float triCenterPos = 0;
        const Triangle& tri = triangles[i];
        
        if (splitAxis == 0) {
            triCenterPos = (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0f;
        } else if (splitAxis == 1) {
            triCenterPos = (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0f;
        } else {
            triCenterPos = (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0f;
        }
        
        if (triCenterPos < splitPos) {
            // Keep in left partition
            splitIdx++;
        }
    }
    
    // If split didn't work well, force it
    if (splitIdx == node.triStart || splitIdx == node.triStart + node.triCount) {
        splitIdx = node.triStart + node.triCount / 2;
    }
    
    // Create child nodes
    int leftCount = splitIdx - node.triStart;
    int rightCount = node.triCount - leftCount;
    
    BuildNode leftChild, rightChild;
    leftChild.triStart = node.triStart;
    leftChild.triCount = leftCount;
    leftChild.childLeft = -1;
    leftChild.childRight = -1;
    
    rightChild.triStart = splitIdx;
    rightChild.triCount = rightCount;
    rightChild.childLeft = -1;
    rightChild.childRight = -1;
    
    // Calculate AABBs
    leftChild.aabbMin = triangles[leftChild.triStart].v0;
    leftChild.aabbMax = triangles[leftChild.triStart].v0;
    for (int i = leftChild.triStart; i < leftChild.triStart + leftChild.triCount; i++) {
        const Triangle& tri = triangles[i];
        leftChild.aabbMin.x = fminf(leftChild.aabbMin.x, fminf(tri.v0.x, fminf(tri.v1.x, tri.v2.x)));
        leftChild.aabbMin.y = fminf(leftChild.aabbMin.y, fminf(tri.v0.y, fminf(tri.v1.y, tri.v2.y)));
        leftChild.aabbMin.z = fminf(leftChild.aabbMin.z, fminf(tri.v0.z, fminf(tri.v1.z, tri.v2.z)));
        leftChild.aabbMax.x = fmaxf(leftChild.aabbMax.x, fmaxf(tri.v0.x, fmaxf(tri.v1.x, tri.v2.x)));
        leftChild.aabbMax.y = fmaxf(leftChild.aabbMax.y, fmaxf(tri.v0.y, fmaxf(tri.v1.y, tri.v2.y)));
        leftChild.aabbMax.z = fmaxf(leftChild.aabbMax.z, fmaxf(tri.v0.z, fmaxf(tri.v1.z, tri.v2.z)));
    }
    
    rightChild.aabbMin = triangles[rightChild.triStart].v0;
    rightChild.aabbMax = triangles[rightChild.triStart].v0;
    for (int i = rightChild.triStart; i < rightChild.triStart + rightChild.triCount; i++) {
        const Triangle& tri = triangles[i];
        rightChild.aabbMin.x = fminf(rightChild.aabbMin.x, fminf(tri.v0.x, fminf(tri.v1.x, tri.v2.x)));
        rightChild.aabbMin.y = fminf(rightChild.aabbMin.y, fminf(tri.v0.y, fminf(tri.v1.y, tri.v2.y)));
        rightChild.aabbMin.z = fminf(rightChild.aabbMin.z, fminf(tri.v0.z, fminf(tri.v1.z, tri.v2.z)));
        rightChild.aabbMax.x = fmaxf(rightChild.aabbMax.x, fmaxf(tri.v0.x, fmaxf(tri.v1.x, tri.v2.x)));
        rightChild.aabbMax.y = fmaxf(rightChild.aabbMax.y, fmaxf(tri.v0.y, fmaxf(tri.v1.y, tri.v2.y)));
        rightChild.aabbMax.z = fmaxf(rightChild.aabbMax.z, fmaxf(tri.v0.z, fmaxf(tri.v1.z, tri.v2.z)));
    }
    
    node.childLeft = buildNodes.size();
    buildNodes.push_back(leftChild);
    
    node.childRight = buildNodes.size();
    buildNodes.push_back(rightChild);
    
    BuildRecursive(triangles, buildNodes, node.childLeft, depth + 1, maxTrisPerLeaf);
    BuildRecursive(triangles, buildNodes, node.childRight, depth + 1, maxTrisPerLeaf);
    
    return nodeIdx;
}

// ============================================================================
// GPU MESH LOADING
// ============================================================================

void LoadArenaCollisionMeshesToGpu(
    const std::vector<struct CollisionMeshFile>& meshFiles,
    GpuArenaCollisionData* outData,
    cudaStream_t stream
) {
    if (!outData || meshFiles.empty()) {
        return;
    }

    std::vector<CpuBvhBuilder::Triangle> triangles;
    triangles.reserve(32768);

    for (const auto& meshFile : meshFiles) {
        if (meshFile.tris.empty() || meshFile.vertices.empty()) {
            continue;
        }

        for (const auto& cpuTri : meshFile.tris) {
            CpuBvhBuilder::Triangle tri = {};
            tri.v0 = {
                meshFile.vertices[cpuTri.vertexIndexes[0]].x,
                meshFile.vertices[cpuTri.vertexIndexes[0]].y,
                meshFile.vertices[cpuTri.vertexIndexes[0]].z
            };
            tri.v1 = {
                meshFile.vertices[cpuTri.vertexIndexes[1]].x,
                meshFile.vertices[cpuTri.vertexIndexes[1]].y,
                meshFile.vertices[cpuTri.vertexIndexes[1]].z
            };
            tri.v2 = {
                meshFile.vertices[cpuTri.vertexIndexes[2]].x,
                meshFile.vertices[cpuTri.vertexIndexes[2]].y,
                meshFile.vertices[cpuTri.vertexIndexes[2]].z
            };

            GpuVec3 e1 = {
                tri.v1.x - tri.v0.x,
                tri.v1.y - tri.v0.y,
                tri.v1.z - tri.v0.z
            };
            GpuVec3 e2 = {
                tri.v2.x - tri.v0.x,
                tri.v2.y - tri.v0.y,
                tri.v2.z - tri.v0.z
            };

            tri.normal = {
                e1.y * e2.z - e1.z * e2.y,
                e1.z * e2.x - e1.x * e2.z,
                e1.x * e2.y - e1.y * e2.x
            };

            float len = sqrtf(tri.normal.x * tri.normal.x +
                              tri.normal.y * tri.normal.y +
                              tri.normal.z * tri.normal.z);
            if (len > 1e-6f) {
                tri.normal.x /= len;
                tri.normal.y /= len;
                tri.normal.z /= len;
            }
            triangles.push_back(tri);
        }
    }

    if (triangles.empty()) {
        return;
    }

    LoadArenaCollisionMeshesFromTriangles(triangles, outData, stream);
}

void LoadArenaCollisionMeshesFromTriangles(
    const std::vector<CpuBvhBuilder::Triangle>& triangles,
    GpuArenaCollisionData* outData,
    cudaStream_t stream
) {
    if (!outData || triangles.empty()) {
        return;
    }

    // Build BVH for all triangles
    std::vector<CpuBvhBuilder::BvhNode> bvhNodes = CpuBvhBuilder::BuildBvh(triangles, 4);

    // Initialize output data
    outData->triangleCount = triangles.size();
    outData->bvhNodeCount = bvhNodes.size();
    outData->numMeshes = 0;  // No separate meshes, all triangles are one collection
    outData->meshes = nullptr;

    // Allocate GPU unified memory
    cudaError_t allocTriErr = cudaMallocManaged(
        reinterpret_cast<void**>(&outData->triangles),
        triangles.size() * sizeof(GpuTriangle),
        cudaMemAttachGlobal
    );
    if (allocTriErr != cudaSuccess) {
        outData->triangles = nullptr;
        outData->bvhNodes = nullptr;
        outData->triangleCount = 0;
        outData->bvhNodeCount = 0;
        return;
    }

    cudaError_t allocBvhErr = cudaMallocManaged(
        reinterpret_cast<void**>(&outData->bvhNodes),
        bvhNodes.size() * sizeof(GpuBvhNode),
        cudaMemAttachGlobal
    );
    if (allocBvhErr != cudaSuccess) {
        cudaFree(outData->triangles);
        outData->triangles = nullptr;
        outData->bvhNodes = nullptr;
        outData->triangleCount = 0;
        outData->bvhNodeCount = 0;
        return;
    }

    // Copy data to GPU
    if (outData->triangles) {
        memcpy(outData->triangles, triangles.data(), triangles.size() * sizeof(GpuTriangle));
    }
    if (outData->bvhNodes) {
        memcpy(outData->bvhNodes, bvhNodes.data(), bvhNodes.size() * sizeof(GpuBvhNode));
    }

    // Sync with GPU
    if (stream) {
        cudaStreamSynchronize(stream);
    }
}

void UnloadArenaCollisionMeshes(GpuArenaCollisionData* data) {
    if (!data) return;
    
    if (data->triangles) {
        cudaFree(data->triangles);
        data->triangles = nullptr;
    }
    if (data->bvhNodes) {
        cudaFree(data->bvhNodes);
        data->bvhNodes = nullptr;
    }
    if (data->meshes) {
        cudaFree(data->meshes);
        data->meshes = nullptr;
    }
    
    data->triangleCount = 0;
    data->bvhNodeCount = 0;
    data->numMeshes = 0;
}

// ============================================================================
// HELPER FUNCTIONS FOR SPHERE-TRIANGLE COLLISION DETECTION
// ============================================================================

__device__ inline bool SphereTriangleIntersection(
    const GpuVec3& sphereCenter,
    float sphereRadius,
    const GpuTriangle& tri,
    GpuVec3& collisionNormal,
    float& penetration
) {
    // Vector from sphere center to triangle v0
    GpuVec3 v0ToSphere = {
        sphereCenter.x - tri.v0.x,
        sphereCenter.y - tri.v0.y,
        sphereCenter.z - tri.v0.z
    };
    
    // Project sphere center onto triangle plane
    float dist = v0ToSphere.x * tri.normal.x + 
                 v0ToSphere.y * tri.normal.y + 
                 v0ToSphere.z * tri.normal.z;
    
    // Check if within radius
    if (fabsf(dist) > sphereRadius) {
        return false;
    }
    
    // Closest point on plane
    GpuVec3 closestPoint = {
        sphereCenter.x - dist * tri.normal.x,
        sphereCenter.y - dist * tri.normal.y,
        sphereCenter.z - dist * tri.normal.z
    };
    
    // Check if point is inside triangle (barycentric coordinates)
    GpuVec3 edge0 = { tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z };
    GpuVec3 edge1 = { tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z };
    GpuVec3 c0 = { closestPoint.x - tri.v0.x, closestPoint.y - tri.v0.y, closestPoint.z - tri.v0.z };
    
    float dot00 = edge0.x * edge0.x + edge0.y * edge0.y + edge0.z * edge0.z;
    float dot01 = edge0.x * edge1.x + edge0.y * edge1.y + edge0.z * edge1.z;
    float dot02 = edge0.x * c0.x + edge0.y * c0.y + edge0.z * c0.z;
    float dot11 = edge1.x * edge1.x + edge1.y * edge1.y + edge1.z * edge1.z;
    float dot12 = edge1.x * c0.x + edge1.y * c0.y + edge1.z * c0.z;
    
    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01 + 1e-6f);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    
    if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f) {
        // Point is inside triangle
        collisionNormal = tri.normal;
        if (dist < 0) {
            collisionNormal.x = -collisionNormal.x;
            collisionNormal.y = -collisionNormal.y;
            collisionNormal.z = -collisionNormal.z;
            penetration = sphereRadius + dist;
        } else {
            penetration = sphereRadius - dist;
        }
        return true;
    }
    
    // Find closest edge/vertex
    float minDist = sphereRadius * sphereRadius;
    float closestDist = minDist;
    
    // Check edges and vertices (simplified - just check distance to vertices for now)
    for (int i = 0; i < 3; i++) {
        GpuVec3 v = (i == 0) ? tri.v0 : ((i == 1) ? tri.v1 : tri.v2);
        GpuVec3 toV = { v.x - sphereCenter.x, v.y - sphereCenter.y, v.z - sphereCenter.z };
        float d = toV.x * toV.x + toV.y * toV.y + toV.z * toV.z;
        if (d < closestDist) {
            closestDist = d;
            collisionNormal = toV;
        }
    }
    
    if (closestDist < minDist) {
        float d = sqrtf(closestDist);
        if (d > 1e-6f) {
            collisionNormal.x /= d;
            collisionNormal.y /= d;
            collisionNormal.z /= d;
        }
        penetration = sphereRadius - sqrtf(closestDist);
        return true;
    }
    
    return false;
}

__device__ inline bool SphereAABBIntersection(
    const GpuVec3& sphereCenter,
    float sphereRadius,
    const GpuVec3& aabbMin,
    const GpuVec3& aabbMax
) {
    float dx = fmaxf(0.0f, fmaxf(aabbMin.x - sphereCenter.x, sphereCenter.x - aabbMax.x));
    float dy = fmaxf(0.0f, fmaxf(aabbMin.y - sphereCenter.y, sphereCenter.y - aabbMax.y));
    float dz = fmaxf(0.0f, fmaxf(aabbMin.z - sphereCenter.z, sphereCenter.z - aabbMax.z));
    return (dx * dx + dy * dy + dz * dz) <= (sphereRadius * sphereRadius);
}

// Ray-triangle intersection (Möller–Trumbore algorithm)
__device__ inline bool RayTriangleIntersection(
    const GpuVec3& rayOrigin,
    const GpuVec3& rayDir,
    float rayLength,
    const GpuTriangle& tri,
    float& t,  // Distance along ray to hit point
    GpuVec3& hitNormal
) {
    constexpr float EPSILON = 1e-6f;
    
    // Edge vectors
    GpuVec3 edge1 = { tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z };
    GpuVec3 edge2 = { tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z };
    
    // Calculate determinant
    GpuVec3 pVec = {
        rayDir.y * edge2.z - rayDir.z * edge2.y,
        rayDir.z * edge2.x - rayDir.x * edge2.z,
        rayDir.x * edge2.y - rayDir.y * edge2.x
    };
    
    float det = edge1.x * pVec.x + edge1.y * pVec.y + edge1.z * pVec.z;
    
    // Ray parallel to triangle
    if (fabsf(det) < EPSILON) {
        return false;
    }
    
    float invDet = 1.0f / det;
    
    // Calculate u parameter
    GpuVec3 tVec = { rayOrigin.x - tri.v0.x, rayOrigin.y - tri.v0.y, rayOrigin.z - tri.v0.z };
    float u = (tVec.x * pVec.x + tVec.y * pVec.y + tVec.z * pVec.z) * invDet;
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    // Calculate v parameter
    GpuVec3 qVec = {
        tVec.y * edge1.z - tVec.z * edge1.y,
        tVec.z * edge1.x - tVec.x * edge1.z,
        tVec.x * edge1.y - tVec.y * edge1.x
    };
    
    float v = (rayDir.x * qVec.x + rayDir.y * qVec.y + rayDir.z * qVec.z) * invDet;
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    // Calculate t (distance along ray)
    t = (edge2.x * qVec.x + edge2.y * qVec.y + edge2.z * qVec.z) * invDet;
    
    if (t < 0.0f || t > rayLength) {
        return false;
    }
    
    hitNormal = tri.normal;
    return true;
}

// Ray-AABB intersection
__device__ inline bool RayAABBIntersection(
    const GpuVec3& rayOrigin,
    const GpuVec3& rayDir,
    float rayLength,
    const GpuVec3& aabbMin,
    const GpuVec3& aabbMax
) {
    float tmin = 0.0f;
    float tmax = rayLength;
    
    for (int i = 0; i < 3; i++) {
        float origin = (i == 0) ? rayOrigin.x : ((i == 1) ? rayOrigin.y : rayOrigin.z);
        float dir = (i == 0) ? rayDir.x : ((i == 1) ? rayDir.y : rayDir.z);
        float bmin = (i == 0) ? aabbMin.x : ((i == 1) ? aabbMin.y : aabbMin.z);
        float bmax = (i == 0) ? aabbMax.x : ((i == 1) ? aabbMax.y : aabbMax.z);
        
        if (fabsf(dir) < 1e-6f) {
            // Ray is parallel to slab
            if (origin < bmin || origin > bmax) {
                return false;
            }
        } else {
            float invDir = 1.0f / dir;
            float t1 = (bmin - origin) * invDir;
            float t2 = (bmax - origin) * invDir;
            
            if (t1 > t2) {
                float tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            
            tmin = fmaxf(tmin, t1);
            tmax = fminf(tmax, t2);
            
            if (tmin > tmax) {
                return false;
            }
        }
    }
    
    return true;
}

// ============================================================================
// GPU MESH COLLISION KERNELS
// ============================================================================

CUDA_KERNEL void MeshCollisionKernelImpl(
    GpuBallState* balls,
    int numBalls,
    const GpuTriangle* triangles,
    const GpuBvhNode* bvhNodes,
    int bvhRootIdx
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBalls) return;
    
    GpuBallState& ball = balls[idx];
    
    // Simple BVH traversal - stack-based approach
    int nodeStack[64];  // Stack for BVH traversal
    int stackTop = 0;
    nodeStack[stackTop++] = bvhRootIdx;
    
    constexpr float BALL_RADIUS = 92.15f;  // From RLConst
    float collisionResponseDamping = 0.96f;  // Slight energy loss on collision
    
    // Traverse BVH
    while (stackTop > 0) {
        int nodeIdx = nodeStack[--stackTop];
        const auto& node = bvhNodes[nodeIdx];
        
        // Check AABB intersection
        if (!SphereAABBIntersection(ball.pos, BALL_RADIUS, node.aabbMin, node.aabbMax)) {
            continue;
        }
        
        // If leaf node, check triangle collisions
        if (node.triCount > 0 && node.childLeft < 0) {
            for (int i = 0; i < node.triCount; i++) {
                const GpuTriangle& tri = triangles[node.triStart + i];
                GpuVec3 collisionNormal;
                float penetration = 0;
                
                if (SphereTriangleIntersection(ball.pos, BALL_RADIUS, tri, collisionNormal, penetration)) {
                    // Apply collision response
                    if (penetration > 0) {
                        // Push ball out of triangle
                        ball.pos.x += collisionNormal.x * penetration;
                        ball.pos.y += collisionNormal.y * penetration;
                        ball.pos.z += collisionNormal.z * penetration;
                        
                        // Apply restitution
                        float velAlongNormal = ball.vel.x * collisionNormal.x +
                                             ball.vel.y * collisionNormal.y +
                                             ball.vel.z * collisionNormal.z;
                        
                        if (velAlongNormal < 0) {
                            constexpr float RESTITUTION = 0.6f;
                            float impulse = -(1.0f + RESTITUTION) * velAlongNormal;
                            
                            ball.vel.x += collisionNormal.x * impulse;
                            ball.vel.y += collisionNormal.y * impulse;
                            ball.vel.z += collisionNormal.z * impulse;
                            
                            // Apply damping
                            ball.vel.x *= collisionResponseDamping;
                            ball.vel.y *= collisionResponseDamping;
                            ball.vel.z *= collisionResponseDamping;
                        }
                    }
                }
            }
        } else if (node.childLeft >= 0) {
            // Push children onto stack
            if (stackTop <= 61) {  // Need room for two pushes
                nodeStack[stackTop++] = node.childRight;
                nodeStack[stackTop++] = node.childLeft;
            }
        }
    }
}

void LaunchMeshCollisionKernel(
    GpuBallState* balls,
    int numBalls,
    const GpuArenaCollisionData* arenaCollision,
    float dt,
    cudaStream_t stream
) {
    (void)dt;

    if (!balls || numBalls <= 0 || !arenaCollision || arenaCollision->triangleCount == 0) {
        return;
    }
    
    constexpr int threadsPerBlock = 256;
    int blocksNeeded = (numBalls + threadsPerBlock - 1) / threadsPerBlock;
    
    // Launch kernel for mesh collision
    MeshCollisionKernelImpl<<<blocksNeeded, threadsPerBlock, 0, stream>>>(
        balls,
        numBalls,
        arenaCollision->triangles,
        arenaCollision->bvhNodes,
        0  // BVH root is always index 0
    );
}

// ============================================================================
// GPU WHEEL RAYCASTING KERNEL
// ============================================================================

// ViteLearn-inspired suspension constants
namespace VehicleConst {
    constexpr float SUSPENSION_STIFFNESS = 1000.0f;      // Spring constant
    constexpr float DAMPING_COMPRESSION = 25.0f;         // Compression damping
    constexpr float DAMPING_RELAXATION = 40.0f;          // Expansion damping
    constexpr float BILATERAL_DAMPING = -0.2f;           // Friction damping
}

// Bilateral constraint solver (from ViteLearn)
__device__ inline float ComputeJacobianDiagonal(
    const GpuMat3x3& worldBasis,
    const GpuVec3& relPos,
    const GpuVec3& invInertia,
    float invMass,
    const GpuVec3& axis
) {
    // Cross product: relPos x axis
    GpuVec3 crossed = {
        relPos.y * axis.z - relPos.z * axis.y,
        relPos.z * axis.x - relPos.x * axis.z,
        relPos.x * axis.y - relPos.y * axis.x
    };
    
    // Transform to world space
    GpuVec3 worldCrossed = worldBasis * crossed;
    
    // Apply inertia
    GpuVec3 inertiaApplied = {
        worldCrossed.x * invInertia.x,
        worldCrossed.y * invInertia.y,
        worldCrossed.z * invInertia.z
    };
    
    return invMass + inertiaApplied.dot(worldCrossed);
}

__device__ inline float ResolveSingleBilateral(
    const GpuVec3& carPos,
    const GpuVec3& carVel,
    const GpuVec3& carAngVel,
    const GpuMat3x3& carRot,
    const GpuVec3& invInertia,
    float invMass,
    const GpuVec3& contactPos1,
    const GpuVec3& contactPos2,
    const GpuVec3& normal
) {
    GpuVec3 rel1 = contactPos1 - carPos;
    GpuVec3 vel1 = carVel + GpuVec3{
        carAngVel.y * rel1.z - carAngVel.z * rel1.y,
        carAngVel.z * rel1.x - carAngVel.x * rel1.z,
        carAngVel.x * rel1.y - carAngVel.y * rel1.x
    };
    
    // Ground-side velocity is treated as zero.
    float jacDiag = ComputeJacobianDiagonal(carRot, rel1, invInertia, invMass, normal);
    float jacInv = (jacDiag > 1e-6f) ? (1.0f / jacDiag) : 0.0f;
    float relVel = normal.dot(vel1);
    
    return VehicleConst::BILATERAL_DAMPING * relVel * jacInv;
}

CUDA_KERNEL void WheelRaycastKernelImpl(
    GpuCarState* cars,
    int numCars,
    const GpuTriangle* triangles,
    const GpuBvhNode* bvhNodes,
    int bvhRootIdx,
    float wheelRadius
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car = cars[idx];
    if (car.isDemoed) return;
    
    // Ray cast for each wheel
    for (int wheelIdx = 0; wheelIdx < car.numWheels; wheelIdx++) {
        auto& wheel = car.wheels[wheelIdx];
        
        // Calculate wheel world position (connection point in car space -> world space)
        GpuVec3 wheelPosWorld = car.rotMat * wheel.connectionPoint;
        wheelPosWorld.x += car.pos.x;
        wheelPosWorld.y += car.pos.y;
        wheelPosWorld.z += car.pos.z;
        
        // Ray direction is downward in world space (car's down vector)
        GpuVec3 rayDir = {
            car.rotMat.m[2][0],  // Down = -Z in car space
            car.rotMat.m[2][1],
            car.rotMat.m[2][2]
        };
        
        // Suspension travel parameters (from RLConst)
        constexpr float SUSPENSION_REST_LENGTH = 38.755f;
        constexpr float MAX_SUSPENSION_TRAVEL_CM = 12.0f;
        constexpr float SUSPENSION_SUBTRACTION = 25.0f;
        
        float suspensionTravel = MAX_SUSPENSION_TRAVEL_CM / 100.0f;
        float rayLength = SUSPENSION_REST_LENGTH + suspensionTravel + wheelRadius - SUSPENSION_SUBTRACTION;
        
        // BVH traversal for ray casting
        int nodeStack[64];
        int stackTop = 0;
        nodeStack[stackTop++] = bvhRootIdx;
        
        float closestT = rayLength;
        bool hitFound = false;
        GpuVec3 hitNormal = {0, 0, 1};
        
        while (stackTop > 0) {
            int nodeIdx = nodeStack[--stackTop];
            const auto& node = bvhNodes[nodeIdx];
            
            // Check ray-AABB intersection
            if (!RayAABBIntersection(wheelPosWorld, rayDir, closestT, node.aabbMin, node.aabbMax)) {
                continue;
            }
            
            // If leaf node, check triangle intersections
            if (node.triCount > 0 && node.childLeft < 0) {
                for (int i = 0; i < node.triCount; i++) {
                    const GpuTriangle& tri = triangles[node.triStart + i];
                    float t;
                    GpuVec3 normal;
                    
                    if (RayTriangleIntersection(wheelPosWorld, rayDir, closestT, tri, t, normal)) {
                        if (t < closestT) {
                            closestT = t;
                            hitNormal = normal;
                            hitFound = true;
                        }
                    }
                }
            } else if (node.childLeft >= 0) {
                // Push children onto stack
                if (stackTop <= 61) {
                    nodeStack[stackTop++] = node.childRight;
                    nodeStack[stackTop++] = node.childLeft;
                }
            }
        }
        
        // Update wheel contact state and calculate suspension physics
        if (hitFound) {
            wheel.hasContact = true;
            wheel.isInContactWithWorld = true;
            wheel.contactNormal = hitNormal;
            wheel.contactPoint.x = wheelPosWorld.x + rayDir.x * closestT;
            wheel.contactPoint.y = wheelPosWorld.y + rayDir.y * closestT;
            wheel.contactPoint.z = wheelPosWorld.z + rayDir.z * closestT;
            wheel.hardPoint = wheelPosWorld;
            wheel.wheelDirection = rayDir;
            wheel.suspensionRestLength = SUSPENSION_REST_LENGTH;
            
            // Calculate suspension length (distance traveled minus wheel radius)
            GpuVec3 upDir = {-rayDir.x, -rayDir.y, -rayDir.z};
            GpuVec3 hardToContact = {
                wheel.contactPoint.x - wheelPosWorld.x,
                wheel.contactPoint.y - wheelPosWorld.y,
                wheel.contactPoint.z - wheelPosWorld.z
            };
            float wheelTraceLenSq = hardToContact.dot(upDir);
            wheel.suspensionLength = wheelTraceLenSq - wheelRadius;
            
            // Clamp suspension length
            float minSuspension = SUSPENSION_REST_LENGTH - suspensionTravel;
            float maxSuspension = SUSPENSION_REST_LENGTH + suspensionTravel;
            wheel.suspensionLength = fmaxf(minSuspension, fminf(maxSuspension, wheel.suspensionLength));
            
            // Calculate suspension velocity and inverse contact dot
            GpuVec3 relPos = {
                wheel.contactPoint.x - car.pos.x,
                wheel.contactPoint.y - car.pos.y,
                wheel.contactPoint.z - car.pos.z
            };
            
            // Velocity at contact point = linear_vel + angular_vel x relPos
            wheel.velocityAtContact = car.vel + GpuVec3{
                car.angVel.y * relPos.z - car.angVel.z * relPos.y,
                car.angVel.z * relPos.x - car.angVel.x * relPos.z,
                car.angVel.x * relPos.y - car.angVel.y * relPos.x
            };
            
            float projVel = wheel.contactNormal.dot(wheel.velocityAtContact);
            float denominator = wheel.contactNormal.dot(upDir);
            
            if (denominator > 0.1f) {
                float inv = 1.0f / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            } else {
                wheel.suspensionRelativeVelocity = 0.0f;
                wheel.clippedInvContactDotSuspension = 10.0f;
            }
            
            // Calculate suspension force (ViteLearn formula)
            float springForce = (SUSPENSION_REST_LENGTH - wheel.suspensionLength) * 
                VehicleConst::SUSPENSION_STIFFNESS * wheel.clippedInvContactDotSuspension;
            
            float dampingCoeff = (wheel.suspensionRelativeVelocity < 0.0f) ? 
                VehicleConst::DAMPING_COMPRESSION : VehicleConst::DAMPING_RELAXATION;
            
            float dampingForce = wheel.suspensionRelativeVelocity * dampingCoeff;
            
            wheel.suspensionForce = (springForce - dampingForce);
            if (wheel.suspensionForce < 0.0f) {
                wheel.suspensionForce = 0.0f;
            }
            
        } else {
            wheel.hasContact = false;
            wheel.isInContactWithWorld = false;
            wheel.suspensionLength = SUSPENSION_REST_LENGTH;
            wheel.suspensionForce = 0.0f;
            wheel.suspensionRelativeVelocity = 0.0f;
            wheel.clippedInvContactDotSuspension = 1.0f;
        }
    }
    
    // Update car grounded state (3+ wheels = grounded)
    int wheelsOnGround = 0;
    for (int i = 0; i < car.numWheels; i++) {
        if (car.wheels[i].hasContact) {
            wheelsOnGround++;
        }
    }
    car.isOnGround = (wheelsOnGround >= 3);
}

void LaunchWheelRaycastKernel(
    GpuCarState* cars,
    int numCars,
    const GpuArenaCollisionData* arenaCollision,
    float wheelRadius,
    cudaStream_t stream
) {
    if (!cars || numCars <= 0 || !arenaCollision || arenaCollision->triangleCount == 0) {
        return;
    }
    
    constexpr int threadsPerBlock = 256;
    int blocksNeeded = (numCars + threadsPerBlock - 1) / threadsPerBlock;
    
    // Launch kernel for wheel raycasting
    WheelRaycastKernelImpl<<<blocksNeeded, threadsPerBlock, 0, stream>>>(
        cars,
        numCars,
        arenaCollision->triangles,
        arenaCollision->bvhNodes,
        0,  // BVH root is always index 0
        wheelRadius
    );
}

// ============================================================================
// GPU SUSPENSION FORCE APPLICATION KERNEL
// ============================================================================

CUDA_KERNEL void ApplySuspensionForcesKernelImpl(
    GpuCarState* cars,
    int numCars,
    float deltaTime
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car = cars[idx];
    if (car.isDemoed) return;
    
    constexpr float CAR_MASS = 180.0f;
    constexpr float INV_MASS = 1.0f / CAR_MASS;
    
    // Simplified inertia (TODO: should be from car config)
    GpuVec3 invInertia = {1.0f / 45.0f, 1.0f / 30.0f, 1.0f / 60.0f};
    
    // Apply suspension forces from each wheel
    for (int i = 0; i < car.numWheels; i++) {
        auto& wheel = car.wheels[i];
        if (!wheel.hasContact || wheel.suspensionForce <= 0.0f) continue;
        
        // Force direction is opposite to wheel direction (upward)
        GpuVec3 forceDir = {
            -wheel.wheelDirection.x,
            -wheel.wheelDirection.y,
            -wheel.wheelDirection.z
        };
        
        GpuVec3 impulse = forceDir * (wheel.suspensionForce * deltaTime);
        
        // Apply central impulse
        car.vel.x += impulse.x * INV_MASS;
        car.vel.y += impulse.y * INV_MASS;
        car.vel.z += impulse.z * INV_MASS;
        
        // Apply torque impulse (force applied at wheel contact point)
        GpuVec3 relPos = {
            wheel.contactPoint.x - car.pos.x,
            wheel.contactPoint.y - car.pos.y,
            wheel.contactPoint.z - car.pos.z
        };
        
        // Torque = relPos x force
        GpuVec3 torque = {
            relPos.y * impulse.z - relPos.z * impulse.y,
            relPos.z * impulse.x - relPos.x * impulse.z,
            relPos.x * impulse.y - relPos.y * impulse.x
        };
        
        // Apply angular impulse via inertia tensor
        // For simplified: angVel += inv_inertia * torque
        car.angVel.x += torque.x * invInertia.x;
        car.angVel.y += torque.y * invInertia.y;
        car.angVel.z += torque.z * invInertia.z;
        
        // Apply bilateral friction constraint (lateral/longitudinal)
        if (wheel.isInContactWithWorld) {
            // Calculate forward and side directions
            GpuVec3 forward = car.rotMat * GpuVec3{1, 0, 0};
            GpuVec3 side = car.rotMat * GpuVec3{0, 1, 0};
            
            // Lateral friction
            float lateralImpulse = ResolveSingleBilateral(
                car.pos, car.vel, car.angVel, car.rotMat, invInertia, INV_MASS,
                wheel.contactPoint, wheel.contactPoint, side
            );
            
            GpuVec3 latImp = side * lateralImpulse;
            car.vel.x += latImp.x * INV_MASS;
            car.vel.y += latImp.y * INV_MASS;
            car.vel.z += latImp.z * INV_MASS;
            
            // Longitudinal friction (simplified)
            float longImpulse = ResolveSingleBilateral(
                car.pos, car.vel, car.angVel, car.rotMat, invInertia, INV_MASS,
                wheel.contactPoint, wheel.contactPoint, forward
            );
            
            GpuVec3 longImp = forward * longImpulse;
            car.vel.x += longImp.x * INV_MASS;
            car.vel.y += longImp.y * INV_MASS;
            car.vel.z += longImp.z * INV_MASS;
        }
    }
}

void LaunchApplySuspensionForcesKernel(
    GpuCarState* cars,
    int numCars,
    float deltaTime,
    cudaStream_t stream
) {
    if (!cars || numCars <= 0) return;
    
    constexpr int threadsPerBlock = 256;
    int blocksNeeded = (numCars + threadsPerBlock - 1) / threadsPerBlock;
    
    ApplySuspensionForcesKernelImpl<<<blocksNeeded, threadsPerBlock, 0, stream>>>(
        cars, numCars, deltaTime
    );
}

RS_NS_END

#endif // RS_CUDA_ENABLED
