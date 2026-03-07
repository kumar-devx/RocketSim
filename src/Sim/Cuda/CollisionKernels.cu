#ifdef RS_CUDA_ENABLED

#include "CollisionKernels.h"
#include "GpuConstants.cuh"

RS_NS_START

// Simple ball-floor collision (for testing/fallback)
CUDA_KERNEL void BallFloorCollisionKernel(GpuBallState* balls, int numBalls) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBalls) return;
    
    GpuBallState& ball = balls[idx];
    
    float groundZ = 0.0f;
    float ballBottom = ball.pos.z - ball.radius;
    
    if (ballBottom < groundZ) {
        // Ball penetrating floor - apply collision response
        float penetration = groundZ - ballBottom;
        
        // Push ball up
        ball.pos.z += penetration;
        
        // Apply restitution
        if (ball.vel.z < 0) {
            ball.vel.z *= -ball.restitution;
            
            // Apply friction
            float frictionScale = 1.0f - ball.friction * 0.1f;
            ball.vel.x *= frictionScale;
            ball.vel.y *= frictionScale;
        }
    }
}

// Sphere vs Box collision (ball vs car hitbox)
CUDA_DEVICE bool SphereBoxCollision(
    const GpuVec3& spherePos, float sphereRadius,
    const GpuVec3& boxPos, const GpuVec3& boxSize, const GpuMat3x3& boxRot,
    GpuVec3& contactNormal, float& penetrationDepth
) {
    // Transform sphere to box local space
    GpuVec3 relPos = spherePos - boxPos;
    
    // Simplified: AABB collision (ignoring rotation for now)
    GpuVec3 halfExtents = boxSize * 0.5f;
    
    // Find closest point on box to sphere
    GpuVec3 closestPoint;
    closestPoint.x = fmaxf(-halfExtents.x, fminf(halfExtents.x, relPos.x));
    closestPoint.y = fmaxf(-halfExtents.y, fminf(halfExtents.y, relPos.y));
    closestPoint.z = fmaxf(-halfExtents.z, fminf(halfExtents.z, relPos.z));
    
    GpuVec3 delta = relPos - closestPoint;
    float distSq = delta.lengthSq();
    
    if (distSq < sphereRadius * sphereRadius) {
        float dist = sqrtf(distSq);
        contactNormal = (dist > 0.0001f) ? delta.normalized() : GpuVec3{0, 0, 1};
        penetrationDepth = sphereRadius - dist;
        return true;
    }
    
    return false;
}

// Ball-car collision kernel
CUDA_KERNEL void BallCarCollisionKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int numBalls,
    int numCars
) {
    int ballIdx = blockIdx.x;
    int carIdx = threadIdx.x;
    
    if (ballIdx >= numBalls || carIdx >= numCars) return;
    
    GpuBallState& ball = balls[ballIdx];
    GpuCarState& car = cars[carIdx];
    
    if (car.isDemoed) return;
    
    GpuVec3 contactNormal;
    float penetration;
    
    if (SphereBoxCollision(
        ball.pos, ball.radius,
        car.pos, car.hitboxSize, car.rotMat,
        contactNormal, penetration
    )) {
        // Simple collision response
        // Separate objects
        ball.pos = ball.pos + contactNormal * penetration;
        
        // Apply impulse (simplified)
        GpuVec3 relVel = ball.vel - car.vel;
        float velAlongNormal = relVel.dot(contactNormal);
        
        if (velAlongNormal < 0) {
            float restitution = GpuRLConst::CARBALL_COLLISION_RESTITUTION;
            float j = -(1.0f + restitution) * velAlongNormal;
            j /= (1.0f / ball.mass + 1.0f / car.mass);
            
            GpuVec3 impulse = contactNormal * j;
            ball.vel = ball.vel + impulse * (1.0f / ball.mass);
            car.vel = car.vel - impulse * (1.0f / car.mass);
        }
    }
}

// Simple ground detection for cars
CUDA_KERNEL void CarGroundDetectionKernel(GpuCarState* cars, int numCars) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car = cars[idx];
    
    if (car.isDemoed) {
        car.isOnGround = false;
        return;
    }
    
    // Simplified ground detection: if car is low and moving down slowly
    float groundZ = 17.0f; // Approximate car rest height
    float heightAboveGround = car.pos.z - groundZ;
    
    if (heightAboveGround < 50.0f && car.vel.z > -300.0f) {
        car.isOnGround = true;
        
        // Apply ground constraint
        if (car.pos.z < groundZ) {
            car.pos.z = groundZ;
            if (car.vel.z < 0) {
                car.vel.z = 0;
            }
        }
    } else {
        car.isOnGround = false;
    }
}

// Host launchers
void LaunchBallFloorCollisionKernel(GpuBallState* balls, int numBalls, cudaStream_t stream) {
    const int threadsPerBlock = 256;
    const int numBlocks = (numBalls + threadsPerBlock - 1) / threadsPerBlock;
    
    BallFloorCollisionKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(balls, numBalls);
}

void LaunchBallCarCollisionKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int numBalls,
    int numCars,
    cudaStream_t stream
) {
    // Launch with one block per ball, threads per car
    dim3 blocks(numBalls, 1, 1);
    dim3 threads(numCars, 1, 1);
    
    BallCarCollisionKernel<<<blocks, threads, 0, stream>>>(balls, cars, numBalls, numCars);
}

void LaunchCarGroundDetectionKernel(GpuCarState* cars, int numCars, cudaStream_t stream) {
    const int threadsPerBlock = 256;
    const int numBlocks = (numCars + threadsPerBlock - 1) / threadsPerBlock;
    
    CarGroundDetectionKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(cars, numCars);
}

RS_NS_END

#endif // RS_CUDA_ENABLED
