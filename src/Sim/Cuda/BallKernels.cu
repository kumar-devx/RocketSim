#ifdef RS_CUDA_ENABLED

#include "BallKernels.h"
#include "GpuConstants.cuh"

RS_NS_START

// GPU kernel for ball physics integration
CUDA_KERNEL void BallIntegrationKernel(GpuBallState* balls, int numBalls, float dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBalls) return;
    
    GpuBallState& ball = balls[idx];
    
    // Apply gravity
    ball.vel.z += GpuRLConst::GRAVITY_Z * dt;
    
    // Apply drag (linear damping)
    float dragFactor = 1.0f - (ball.drag * dt);
    if (dragFactor < 0.0f) dragFactor = 0.0f;
    
    ball.vel.x *= dragFactor;
    ball.vel.y *= dragFactor;
    ball.vel.z *= dragFactor;
    
    // Integrate velocity -> position
    ball.pos.x += ball.vel.x * dt;
    ball.pos.y += ball.vel.y * dt;
    ball.pos.z += ball.vel.z * dt;
    
    // Update rotation (simplified - proper quaternion integration in future)
    // In full implementation, this would update rotMat using angular velocity
    
    ball.tickCount++;
}

// GPU kernel for velocity limiting
CUDA_KERNEL void BallVelocityLimitKernel(GpuBallState* balls, int numBalls) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBalls) return;
    
    GpuBallState& ball = balls[idx];
    
    // Limit linear velocity
    float velLengthSq = ball.vel.lengthSq();
    float maxSpeedSq = ball.maxSpeed * ball.maxSpeed;
    
    if (velLengthSq > maxSpeedSq) {
        float scale = ball.maxSpeed / sqrtf(velLengthSq);
        ball.vel.x *= scale;
        ball.vel.y *= scale;
        ball.vel.z *= scale;
    }
    
    // Limit angular velocity
    float angVelLengthSq = ball.angVel.lengthSq();
    float maxAngSpeedSq = GpuRLConst::BALL_MAX_ANG_SPEED * GpuRLConst::BALL_MAX_ANG_SPEED;
    
    if (angVelLengthSq > maxAngSpeedSq) {
        float scale = GpuRLConst::BALL_MAX_ANG_SPEED / sqrtf(angVelLengthSq);
        ball.angVel.x *= scale;
        ball.angVel.y *= scale;
        ball.angVel.z *= scale;
    }
}

// Host-callable launcher functions
void LaunchBallPhysicsKernel(GpuBallState* balls, int numBalls, float deltaTime, cudaStream_t stream) {
    const int threadsPerBlock = 256;
    const int numBlocks = (numBalls + threadsPerBlock - 1) / threadsPerBlock;
    
    BallIntegrationKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(balls, numBalls, deltaTime);
    BallVelocityLimitKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(balls, numBalls);
}

void LaunchBallIntegrationKernel(GpuBallState* balls, int numBalls, float deltaTime, cudaStream_t stream) {
    const int threadsPerBlock = 256;
    const int numBlocks = (numBalls + threadsPerBlock - 1) / threadsPerBlock;
    
    BallIntegrationKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(balls, numBalls, deltaTime);
}

RS_NS_END

#endif // RS_CUDA_ENABLED
