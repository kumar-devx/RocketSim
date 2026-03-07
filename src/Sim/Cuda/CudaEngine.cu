#ifdef RS_CUDA_ENABLED

#include "CudaEngine.h"
#include <iostream>

RS_NS_START

CudaEngine::CudaEngine() : enabled_(false), stream_(0) {
}

CudaEngine::~CudaEngine() {
    if (stream_) {
        cudaStreamDestroy(stream_);
    }
}

bool CudaEngine::Initialize() {
    if (!CudaMemoryManager::IsCudaAvailable()) {
        std::cerr << "FATAL: No CUDA devices found! GPU is required for this build." << std::endl;
        std::cerr << "Make sure you have:" << std::endl;
        std::cerr << "  1. Compatible NVIDIA GPU (RTX 2000+)" << std::endl;
        std::cerr << "  2. Updated GPU drivers (545.x+ for CUDA 12.6)" << std::endl;
        std::cerr << "  3. CUDA Toolkit 12.6 installed" << std::endl;
        enabled_ = false;
        return false;
    }

    std::cout << "Initializing CUDA for RocketSim..." << std::endl;
    CudaMemoryManager::PrintDeviceInfo();

    // Create CUDA stream for async operations
    cudaError_t err = cudaStreamCreate(&stream_);
    if (err != cudaSuccess) {
        std::cerr << "FATAL: Failed to create CUDA stream: " << cudaGetErrorString(err) << std::endl;
        enabled_ = false;
        return false;
    }

    // Test GPU memory allocation (MANDATORY)
    void* test_ptr = nullptr;
    err = cudaMalloc(&test_ptr, 1024 * 1024); // 1MB test
    if (err != cudaSuccess) {
        std::cerr << "FATAL: Failed to allocate GPU memory: " << cudaGetErrorString(err) << std::endl;
        std::cerr << "GPU is present but cannot allocate memory!" << std::endl;
        if (stream_) cudaStreamDestroy(stream_);
        stream_ = 0;
        enabled_ = false;
        return false;
    }
    cudaFree(test_ptr);

    enabled_ = true;
    std::cout << "CUDA initialized successfully!" << std::endl;
    std::cout << "GPU-ONLY MODE: CPU fallback disabled" << std::endl;
    return true;
}

void CudaEngine::UpdateArena(
    GpuBallState* ball,
    GpuCarState* cars, int numCars,
    float deltaTime
) {
    if (!enabled_) return;
    
    // Update ball physics
    if (ball) {
        LaunchBallPhysicsKernel(ball, 1, deltaTime, stream_);
    }
    
    // Update car physics (includes boost, jump, air control)
    if (cars && numCars > 0) {
        LaunchCarPhysicsKernel(cars, numCars, deltaTime, stream_);
        
        // Update ground detection
        LaunchCarGroundDetectionKernel(cars, numCars, stream_);
    }
    
    // Collision detection
    if (ball) {
        LaunchBallFloorCollisionKernel(ball, 1, stream_);
        
        if (cars && numCars > 0) {
            LaunchBallCarCollisionKernel(ball, cars, 1, numCars, stream_);
        }
    }
}

void CudaEngine::UpdateArenaBatch(
    GpuBallState* balls,
    GpuCarState* cars,
    int* carCounts,
    int numArenas,
    float deltaTime
) {
    if (!enabled_) return;
    
    // Update all balls in parallel
    LaunchBallPhysicsKernel(balls, numArenas, deltaTime, stream_);
    
    // Calculate total number of cars across all arenas
    int totalCars = 0;
    for (int i = 0; i < numArenas; i++) {
        totalCars += carCounts[i];
    }
    
    if (totalCars > 0) {
        // Update all cars in parallel
        LaunchCarPhysicsKernel(cars, totalCars, deltaTime, stream_);
        LaunchCarGroundDetectionKernel(cars, totalCars, stream_);
    }
    
    // Collision detection
    LaunchBallFloorCollisionKernel(balls, numArenas, stream_);
    
    // Ball-car collisions (more complex in batch mode)
    // For now, skip in batch mode - will implement spatial hashing later
    
    // THIS IS WHERE THE MAGIC HAPPENS!
    // Instead of simulating arenas sequentially on CPU,
    // we simulate ALL of them in parallel on GPU!
}

void CudaEngine::Synchronize() {
    if (enabled_) {
        CUDA_CHECK(cudaStreamSynchronize(stream_));
    }
}

RS_NS_END

#endif // RS_CUDA_ENABLED
