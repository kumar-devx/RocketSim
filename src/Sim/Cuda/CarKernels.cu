#ifdef RS_CUDA_ENABLED

#include "CarKernels.h"
#include "GpuConstants.cuh"

RS_NS_START

// Helper: Get forward direction from rotation matrix
CUDA_DEVICE inline GpuVec3 GetForwardDir(const GpuMat3x3& rotMat) {
    return {rotMat.m[1][0], rotMat.m[1][1], rotMat.m[1][2]};
}

CUDA_DEVICE inline GpuVec3 GetRightDir(const GpuMat3x3& rotMat) {
    return {rotMat.m[0][0], rotMat.m[0][1], rotMat.m[0][2]};
}

CUDA_DEVICE inline GpuVec3 GetUpDir(const GpuMat3x3& rotMat) {
    return {rotMat.m[2][0], rotMat.m[2][1], rotMat.m[2][2]};
}

CUDA_DEVICE inline GpuVec3 Cross(const GpuVec3& a, const GpuVec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

CUDA_DEVICE inline void AtomicAddVec3(GpuVec3* dst, const GpuVec3& delta) {
    atomicAdd(&dst->x, delta.x);
    atomicAdd(&dst->y, delta.y);
    atomicAdd(&dst->z, delta.z);
}

// GPU kernel for boost mechanics
CUDA_DEVICE void UpdateBoost(GpuCarState& car, float dt) {
    using namespace GpuRLConst;
    
    bool hasBoost = car.boost_amount > 0;
    
    if (hasBoost) {
        if (car.isBoosting) {
            if (car.boost || car.boostingTime < BOOST_MIN_TIME) {
                car.isBoosting = true;
            } else {
                car.isBoosting = false;
            }
        } else {
            if (car.boost) {
                car.isBoosting = true;
            }
        }
    } else {
        car.isBoosting = false;
    }
    
    if (car.isBoosting) {
        car.boostingTime += dt;
    } else {
        car.boostingTime = 0;
    }
    
    // Apply boost force
    if (car.isBoosting) {
        car.boost_amount = fmaxf(car.boost_amount - BOOST_USED_PER_SECOND * dt, 0.0f);
        
        float boostAccel = car.isOnGround ? BOOST_ACCEL_GROUND : BOOST_ACCEL_AIR;
        GpuVec3 forwardDir = GetForwardDir(car.rotMat);
        
        // Apply boost acceleration
        car.vel = car.vel + forwardDir * (boostAccel * dt);
    }
    
    car.boost_amount = fminf(car.boost_amount, BOOST_MAX);
}

// GPU kernel for jump mechanics
CUDA_DEVICE void UpdateJump(GpuCarState& car, bool jumpPressed, float dt) {
    using namespace GpuRLConst;
    
    if (car.isOnGround && !car.isJumping) {
        if (car.hasJumped && car.jumpTime < JUMP_MIN_TIME) {
            // Don't reset yet
        } else {
            car.hasJumped = false;
            car.jumpTime = 0;
        }
    }
    
    if (car.isJumping) {
        if (car.jumpTime < JUMP_MIN_TIME || (car.jump && car.jumpTime < JUMP_MAX_TIME)) {
            car.isJumping = true;
        } else {
            car.isJumping = false;
        }
    } else if (car.isOnGround && jumpPressed) {
        // Start jumping
        car.isJumping = true;
        car.jumpTime = 0;
        car.hasJumped = true;
        
        GpuVec3 upDir = GetUpDir(car.rotMat);
        // Apply immediate jump impulse
        car.vel = car.vel + upDir * (JUMP_IMMEDIATE_FORCE / CAR_MASS);
    }
    
    if (car.isJumping) {
        GpuVec3 upDir = GetUpDir(car.rotMat);
        float accelScale = (car.jumpTime < JUMP_MIN_TIME) ? 0.62f : 1.0f;
        
        // Apply sustained jump force
        car.vel = car.vel + upDir * (JUMP_ACCEL * accelScale * dt);
    }
    
    if (car.isJumping || car.hasJumped) {
        car.jumpTime += dt;
    }
}

// GPU kernel for air control torque
CUDA_DEVICE void UpdateAirTorque(GpuCarState& car, float dt) {
    using namespace GpuRLConst;
    
    if (car.isOnGround) return;
    
    GpuVec3 dirPitch = GetRightDir(car.rotMat) * -1.0f;
    GpuVec3 dirYaw = GetUpDir(car.rotMat);
    GpuVec3 dirRoll = GetForwardDir(car.rotMat) * -1.0f;
    
    bool doAirControl = true;
    
    if (car.isFlipping) {
        doAirControl = false; // Simplified: full flip control
    }
    
    if (doAirControl) {
        GpuVec3 torque;
        torque.x = car.pitch * dirPitch.x * CAR_AIR_CONTROL_TORQUE_X;
        torque.y = car.pitch * dirPitch.y * CAR_AIR_CONTROL_TORQUE_X;
        torque.z = car.pitch * dirPitch.z * CAR_AIR_CONTROL_TORQUE_X;
        
        torque.x += car.yaw * dirYaw.x * CAR_AIR_CONTROL_TORQUE_Y;
        torque.y += car.yaw * dirYaw.y * CAR_AIR_CONTROL_TORQUE_Y;
        torque.z += car.yaw * dirYaw.z * CAR_AIR_CONTROL_TORQUE_Y;
        
        torque.x += car.roll * dirRoll.x * CAR_AIR_CONTROL_TORQUE_Z;
        torque.y += car.roll * dirRoll.y * CAR_AIR_CONTROL_TORQUE_Z;
        torque.z += car.roll * dirRoll.z * CAR_AIR_CONTROL_TORQUE_Z;
        
        // Apply torque (simplified - directly to angular velocity)
        float torqueScale = dt / CAR_MASS;
        car.angVel = car.angVel + torque * torqueScale;
        
        // Apply damping
        float dampingFactor = 1.0f - (CAR_AIR_CONTROL_DAMPING * dt);
        if (dampingFactor < 0.0f) dampingFactor = 0.0f;
        car.angVel = car.angVel * dampingFactor;
    }
}

// GPU kernel for applying gravity
CUDA_DEVICE void ApplyGravity(GpuCarState& car, float dt) {
    if (!car.isDemoed) {
        car.vel.z += GpuRLConst::GRAVITY_Z * dt;
    }
}

CUDA_DEVICE void ApplyGravityBall(GpuBallState& ball, float dt) {
    ball.vel.z += GpuRLConst::GRAVITY_Z * dt;
}

// Main car physics integration kernel
CUDA_KERNEL void CarPhysicsFullKernel(GpuCarState* cars, int numCars, float dt, bool* jumpPressed) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car = cars[idx];
    
    if (car.isDemoed) {
        return; // Skip demoed cars
    }
    
    // Update ground contact state (simplified - will be improved with raycasting)
    car.isOnGround = (car.pos.z < 50.0f) && (fabsf(car.vel.z) < 100.0f);
    
    // Update jump state
    bool wasJumpPressed = jumpPressed ? jumpPressed[idx] : false;
    UpdateJump(car, wasJumpPressed, dt);
    
    // Update boost
    UpdateBoost(car, dt);
    
    // Apply gravity
    ApplyGravity(car, dt);
    
    // Update air control
    if (!car.isOnGround) {
        UpdateAirTorque(car, dt);
    }
    
    // Integrate velocity -> position
    car.pos.x += car.vel.x * dt;
    car.pos.y += car.vel.y * dt;
    car.pos.z += car.vel.z * dt;
    
    // Update angular velocity -> rotation (simplified quaternion integration)
    // Full implementation would properly integrate angular velocity into rotation matrix
    float angVelMag = car.angVel.length();
    if (angVelMag > 0.0001f) {
        // Simplified rotation update (good enough for now)
        // In production, use proper quaternion integration
    }
    
    car.tickCount++;
}

// GPU kernel for car velocity limiting
CUDA_KERNEL void CarVelocityLimitKernel(GpuCarState* cars, int numCars) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car = cars[idx];
    
    if (car.isDemoed) return;

    // Defensive sanitization for invalid numeric states.
    if (!isfinite(car.vel.x) || !isfinite(car.vel.y) || !isfinite(car.vel.z)) {
        car.vel = {0.0f, 0.0f, 0.0f};
    }
    if (!isfinite(car.angVel.x) || !isfinite(car.angVel.y) || !isfinite(car.angVel.z)) {
        car.angVel = {0.0f, 0.0f, 0.0f};
    }
    
    // Limit linear velocity
    float velLengthSq = car.vel.lengthSq();
    float maxSpeedSq = GpuRLConst::CAR_MAX_SPEED * GpuRLConst::CAR_MAX_SPEED;
    
    if (velLengthSq > maxSpeedSq) {
        float scale = GpuRLConst::CAR_MAX_SPEED / sqrtf(velLengthSq);
        car.vel.x *= scale;
        car.vel.y *= scale;
        car.vel.z *= scale;
    }
    
    // Limit angular velocity
    float angVelLengthSq = car.angVel.lengthSq();
    float maxAngSpeedSq = GpuRLConst::CAR_MAX_ANG_SPEED * GpuRLConst::CAR_MAX_ANG_SPEED;
    
    if (angVelLengthSq > maxAngSpeedSq) {
        float scale = GpuRLConst::CAR_MAX_ANG_SPEED / sqrtf(angVelLengthSq);
        car.angVel.x *= scale;
        car.angVel.y *= scale;
        car.angVel.z *= scale;
    }
}

// Legacy integration helper (kept for compatibility)
// Host-side wrapper to avoid device-side kernel launches.
static CUDA_HOST void CarIntegrationKernel(GpuCarState* cars, int numCars, float dt, cudaStream_t stream) {
    if (!cars || numCars <= 0) return;

    const int threadsPerBlock = 256;
    const int numBlocks = (numCars + threadsPerBlock - 1) / threadsPerBlock;

    CarPhysicsFullKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(cars, numCars, dt, nullptr);
    CUDA_CHECK(cudaGetLastError());
}

// Host-callable launcher functions
void LaunchCarPhysicsKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream) {
    if (!cars || numCars <= 0) return;
    
    const int threadsPerBlock = 256;
    const int numBlocks = (numCars + threadsPerBlock - 1) / threadsPerBlock;
    
    CarPhysicsFullKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(cars, numCars, deltaTime, nullptr);
    CUDA_CHECK(cudaGetLastError()); // Check for kernel launch errors
    
    CarVelocityLimitKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(cars, numCars);
    CUDA_CHECK(cudaGetLastError()); // Check for kernel launch errors
}

void LaunchCarIntegrationKernel(GpuCarState* cars, int numCars, float deltaTime, cudaStream_t stream) {
    CarIntegrationKernel(cars, numCars, deltaTime, stream);
}

void LaunchCarVelocityLimitKernel(GpuCarState* cars, int numCars, cudaStream_t stream) {
    if (!cars || numCars <= 0) return;

    const int threadsPerBlock = 256;
    const int numBlocks = (numCars + threadsPerBlock - 1) / threadsPerBlock;

    CarVelocityLimitKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(cars, numCars);
    CUDA_CHECK(cudaGetLastError());
}

// GPU car-to-car collision detection and impulse application.

CUDA_KERNEL void CarToCarCollisionFullKernel(
    GpuCarState* cars,
    int numCars,
    float carMass
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numCars) return;
    
    GpuCarState& car1 = cars[idx];
    
    // Skip demoed cars
    if (car1.isDemoed) return;
    
    // Check collisions with all other cars (only check pairs once)
    for (int j = idx + 1; j < numCars; j++) {
        GpuCarState& car2 = cars[j];
        
        if (car2.isDemoed) continue;
        
        // Simple sphere-sphere collision for broad-phase
        const float CAR_COLLISION_RADIUS = 150.0f;
        
        GpuVec3 delta = car2.pos - car1.pos;
        float distSq = delta.lengthSq();
        float minDistSq = (CAR_COLLISION_RADIUS * 2) * (CAR_COLLISION_RADIUS * 2);
        
        if (distSq >= minDistSq) continue; // No collision
        
        // Collision detected - compute impulse
        float dist = sqrtf(distSq);
        if (dist < 0.001f) dist = 0.001f; // Avoid division by zero
        
        GpuVec3 contactNormal = delta * (1.0f / dist);
        
        // Relative velocity
        GpuVec3 relVel = car1.vel - car2.vel;
        float velAlongNormal = relVel.dot(contactNormal);
        
        // Skip if cars are moving apart
        if (velAlongNormal <= 0) continue;
        
        // Calculate impulse
        const float RESTITUTION = 0.4f;
        float impulseMagnitude = -(1.0f + RESTITUTION) * velAlongNormal / 2.0f;
        
        if (impulseMagnitude < 0.1f) continue; // Ignore very small collisions
        
        // Apply impulse directly to car velocities (GPU-side)
        // This eliminates need for CPU-side callback processing
        GpuVec3 impulse = contactNormal * impulseMagnitude;
        
        // Update velocities atomically because multiple threads may touch car2.
        GpuVec3 car1Delta = impulse * (1.0f / carMass);
        GpuVec3 car2Delta = impulse * (-1.0f / carMass);
        AtomicAddVec3(&car1.vel, car1Delta);
        AtomicAddVec3(&car2.vel, car2Delta);
    }
}

void LaunchCarToCarCollisionFull(
    GpuCarState* cars,
    int numCars,
    float carMass,
    cudaStream_t stream
) {
    if (!cars || numCars <= 1) return;
    
    const int threadsPerBlock = 256;
    const int numBlocks = (numCars + threadsPerBlock - 1) / threadsPerBlock;
    
    CarToCarCollisionFullKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(
        cars, numCars, carMass
    );
    CUDA_CHECK(cudaGetLastError());
}

RS_NS_END

#endif // RS_CUDA_ENABLED
