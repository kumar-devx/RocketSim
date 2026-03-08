#ifdef RS_CUDA_ENABLED

#include "ArenaBatch.h"
#include "../Arena/Arena.h"
#include "../Ball/Ball.h"
#include "../Car/Car.h"
#include "CudaEngine.h"
#include "../../RocketSim.h"

RS_NS_START

// Declare the kernel (defined in ArenaBatch.cu)
extern void LaunchBatchArenaPhysicsKernel(
    GpuBallState* balls,
    GpuCarState* cars,
    int* carOffsets,
    int numArenas,
    float deltaTime
);

ArenaBatch::ArenaBatch() 
    : totalCarsCapacity_(0), needsReallocation_(true) {
}

ArenaBatch::~ArenaBatch() {
    Clear();
}

void ArenaBatch::AddArena(Arena* arena) {
    arenas_.push_back(arena);
    needsReallocation_ = true;
}

void ArenaBatch::RemoveArena(Arena* arena) {
    auto it = std::find(arenas_.begin(), arenas_.end(), arena);
    if (it != arenas_.end()) {
        arenas_.erase(it);
        needsReallocation_ = true;
    }
}

void ArenaBatch::Clear() {
    arenas_.clear();
    needsReallocation_ = true;
}

void ArenaBatch::AllocateGPUBuffers() {
    if (!needsReallocation_ && !arenas_.empty()) return;
    
    int numArenas = static_cast<int>(arenas_.size());
    if (numArenas == 0) return;
    
    // Count total cars
    int totalCars = 0;
    for (Arena* arena : arenas_) {
        totalCars += static_cast<int>(arena->GetCars().size());
    }
    
    // Allocate buffers
    gpuBalls_ = GpuBuffer<GpuBallState>(numArenas, true);
    
    if (totalCars > totalCarsCapacity_) {
        gpuCars_ = GpuBuffer<GpuCarState>(totalCars, true);
        totalCarsCapacity_ = totalCars;
    }
    
    carOffsets_ = GpuBuffer<int>(numArenas, true);
    
    needsReallocation_ = false;
}

void ArenaBatch::SyncToGPU() {
    int numArenas = static_cast<int>(arenas_.size());
    if (numArenas == 0) return;
    
    int carOffset = 0;
    
    for (int i = 0; i < numArenas; i++) {
        Arena* arena = arenas_[i];
        
        // Sync ball
        BallState ballState = arena->ball->GetState();
        GpuBallState& gpuBall = gpuBalls_.get()[i];
        
        gpuBall.pos = ToGpuVec3(ballState.pos);
        gpuBall.vel = ToGpuVec3(ballState.vel);
        gpuBall.angVel = ToGpuVec3(ballState.angVel);
        gpuBall.rotMat = ToGpuMat3x3(ballState.rotMat);
        gpuBall.radius = arena->_mutatorConfig.ballRadius;
        gpuBall.mass = arena->_mutatorConfig.ballMass;
        gpuBall.drag = arena->_mutatorConfig.ballDrag;
        gpuBall.friction = arena->_mutatorConfig.ballWorldFriction;
        gpuBall.restitution = arena->_mutatorConfig.ballWorldRestitution;
        gpuBall.maxSpeed = arena->_mutatorConfig.ballMaxSpeed;
        gpuBall.tickCount = arena->tickCount;
        
        // Record car offset for this arena
        carOffsets_.get()[i] = carOffset;
        
        // Sync cars
        for (Car* car : arena->GetCars()) {
            CarState carState = car->GetState();
            GpuCarState& gpuCar = gpuCars_.get()[carOffset];
            
            gpuCar.pos = ToGpuVec3(carState.pos);
            gpuCar.vel = ToGpuVec3(carState.vel);
            gpuCar.angVel = ToGpuVec3(carState.angVel);
            gpuCar.rotMat = ToGpuMat3x3(carState.rotMat);
            gpuCar.isDemoed = carState.isDemoed;
            gpuCar.boost_amount = carState.boost;
            gpuCar.mass = arena->_mutatorConfig.carMass;
            
            carOffset++;
        }
    }
}

void ArenaBatch::SyncFromGPU() {
    int numArenas = static_cast<int>(arenas_.size());
    if (numArenas == 0) return;
    
    int carOffset = 0;
    
    for (int i = 0; i < numArenas; i++) {
        Arena* arena = arenas_[i];
        
        // Sync ball back
        const GpuBallState& gpuBall = gpuBalls_.get()[i];
        BallState ballState;
        ballState.pos = FromGpuVec3(gpuBall.pos);
        ballState.vel = FromGpuVec3(gpuBall.vel);
        ballState.angVel = FromGpuVec3(gpuBall.angVel);
        ballState.rotMat = FromGpuMat3x3(gpuBall.rotMat);
        arena->ball->SetState(ballState);
        
        // Sync cars back
        for (Car* car : arena->GetCars()) {
            const GpuCarState& gpuCar = gpuCars_.get()[carOffset];
            
            CarState carState = car->GetState();
            carState.pos = FromGpuVec3(gpuCar.pos);
            carState.vel = FromGpuVec3(gpuCar.vel);
            carState.angVel = FromGpuVec3(gpuCar.angVel);
            carState.rotMat = FromGpuMat3x3(gpuCar.rotMat);
            carState.boost = gpuCar.boost_amount;
            car->SetState(carState);
            
            carOffset++;
        }
    }
}

void ArenaBatch::StepAll(int ticksToSimulate) {
    auto* cudaEngine = RocketSim::GetCudaEngine();
    if (!cudaEngine || !cudaEngine->IsEnabled()) {
        RS_ERR_CLOSE("ArenaBatch requires GPU acceleration! CUDA must be enabled.");
    }

    int numArenas = static_cast<int>(arenas_.size());
    if (numArenas == 0) return;

    // Allocate GPU buffers if needed
    AllocateGPUBuffers();

    for (int tick = 0; tick < ticksToSimulate; tick++) {
        // Sync all arenas to GPU
        SyncToGPU();
        
        // Launch batch kernel via wrapper function
        float deltaTime = arenas_[0]->tickTime;
        LaunchBatchArenaPhysicsKernel(
            gpuBalls_.get(),
            gpuCars_.get(),
            carOffsets_.get(),
            numArenas,
            deltaTime
        );
        
        // Wait for GPU
        cudaEngine->Synchronize();
        
        // Sync results back to CPU
        SyncFromGPU();
        
        // Update tick counts
        for (Arena* arena : arenas_) {
            arena->tickCount++;
        }
    }
}

RS_NS_END

#endif // RS_CUDA_ENABLED
