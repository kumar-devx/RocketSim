#ifdef RS_CUDA_ENABLED

#include "ArenaBatch.h"
#include "GpuConstants.cuh"
#include "CollisionKernels.h"

RS_NS_START

// CUDA kernel for batch arena update (optimized version)
CUDA_KERNEL void BatchArenaPhysicsKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int* carOffsets,
    int numArenas,
    float deltaTime
) {
    int arenaIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (arenaIdx >= numArenas) return;

    GpuBallState& ball = balls[arenaIdx];

    // Apply gravity to ball
    ball.vel.z += GpuRLConst::GRAVITY_Z * deltaTime;

    // Apply drag
    float dragFactor = 1.0f - (ball.drag * deltaTime);
    if (dragFactor < 0.0f) dragFactor = 0.0f;

    ball.vel.x *= dragFactor;
    ball.vel.y *= dragFactor;
    ball.vel.z *= dragFactor;

    // Integrate position
    ball.pos.x += ball.vel.x * deltaTime;
    ball.pos.y += ball.vel.y * deltaTime;
    ball.pos.z += ball.vel.z * deltaTime;

    // Limit velocity
    float velLengthSq = ball.vel.lengthSq();
    float maxSpeedSq = ball.maxSpeed * ball.maxSpeed;
    if (velLengthSq > maxSpeedSq) {
        float scale = ball.maxSpeed / sqrtf(velLengthSq);
        ball.vel.x *= scale;
        ball.vel.y *= scale;
        ball.vel.z *= scale;
    }

    // Simple floor collision
    float groundZ = 0.0f;
    float ballBottom = ball.pos.z - ball.radius;
    if (ballBottom < groundZ) {
        ball.pos.z += (groundZ - ballBottom);
        if (ball.vel.z < 0) {
            ball.vel.z *= -ball.restitution;
        }
    }

    // Update cars for this arena
    int carStart = carOffsets[arenaIdx];
    int carEnd = (arenaIdx < numArenas - 1) ? carOffsets[arenaIdx + 1] : carStart + 1;

    for (int carIdx = carStart; carIdx < carEnd; carIdx++) {
        GpuCarState& car = cars[carIdx];

        if (car.isDemoed) continue;

        // Apply gravity
        car.vel.z += GpuRLConst::GRAVITY_Z * deltaTime;

        // Integrate position
        car.pos.x += car.vel.x * deltaTime;
        car.pos.y += car.vel.y * deltaTime;
        car.pos.z += car.vel.z * deltaTime;

        // Ground detection (simplified)
        float carGroundZ = 17.0f;
        if (car.pos.z < carGroundZ + 50.0f) {
            car.isOnGround = true;
            if (car.pos.z < carGroundZ) {
                car.pos.z = carGroundZ;
                if (car.vel.z < 0) car.vel.z = 0;
            }
        } else {
            car.isOnGround = false;
        }

        // Limit velocity
        const float CAR_MAX_SPEED = 2300.0f;
        float carVelLengthSq = car.vel.lengthSq();
        float carMaxSpeedSq = CAR_MAX_SPEED * CAR_MAX_SPEED;
        if (carVelLengthSq > carMaxSpeedSq) {
            float scale = CAR_MAX_SPEED / sqrtf(carVelLengthSq);
            car.vel.x *= scale;
            car.vel.y *= scale;
            car.vel.z *= scale;
        }
    }

    ball.tickCount++;
}

// Host function to launch the kernel
void LaunchBatchArenaPhysicsKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int* carOffsets,
    int numArenas,
    float deltaTime
) {
    const int threadsPerBlock = 256;
    const int numBlocks = (numArenas + threadsPerBlock - 1) / threadsPerBlock;

    BatchArenaPhysicsKernel<<<numBlocks, threadsPerBlock>>>(
        balls, cars, carOffsets, numArenas, deltaTime
    );
}

RS_NS_END

#endif // RS_CUDA_ENABLED
