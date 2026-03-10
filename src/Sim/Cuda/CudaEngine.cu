#ifdef RS_CUDA_ENABLED

#include "CudaEngine.h"
#include "GpuPhysicsPipeline.h"
#include <iostream>

RS_NS_START

CudaEngine::CudaEngine() : enabled_(false), deviceId_(0), stream_(0) {
}

CudaEngine::~CudaEngine() {
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

bool CudaEngine::Initialize() {
    if (!CudaMemoryManager::IsCudaAvailable()) {
        std::cerr << "FATAL: No CUDA devices found! GPU is required for this build." << std::endl;
        std::cerr << "Make sure you have:" << std::endl;
        std::cerr << "  1. Compatible NVIDIA GPU (RTX 2000+)" << std::endl;
        std::cerr << "  2. Updated NVIDIA driver" << std::endl;
        std::cerr << "  3. Matching CUDA toolkit/runtime for your GPU architecture" << std::endl;
        enabled_ = false;
        return false;
    }

    std::cout << "Initializing CUDA for RocketSim..." << std::endl;
    CudaMemoryManager::PrintDeviceInfo();

    int activeDevice = 0;
    CUDA_CHECK(cudaGetDevice(&activeDevice));
    deviceId_ = activeDevice;

    cudaDeviceProp prop;
    CUDA_CHECK(cudaGetDeviceProperties(&prop, activeDevice));

    int runtimeVersion = 0;
    int driverVersion = 0;
    CUDA_CHECK(cudaRuntimeGetVersion(&runtimeVersion));
    CUDA_CHECK(cudaDriverGetVersion(&driverVersion));

    // Compute capability 12.x GPUs (e.g., RTX 50-series) require CUDA 12.8+ for kernel support.
    if (prop.major >= 12 && runtimeVersion < 12080) {
        std::cerr << "FATAL: GPU " << prop.name << " reports compute capability "
                  << prop.major << "." << prop.minor << ", but CUDA runtime is too old (" 
                  << runtimeVersion << ")." << std::endl;
        std::cerr << "Rebuild with CUDA 12.8+ and sm_120 target support." << std::endl;
        enabled_ = false;
        return false;
    }

    if (runtimeVersion > driverVersion) {
        std::cerr << "WARNING: CUDA runtime version (" << runtimeVersion
                  << ") exceeds driver supported version (" << driverVersion << ")." << std::endl;
        std::cerr << "Kernel launches may fail until the NVIDIA driver is updated." << std::endl;
    }

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
        if (stream_) {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
        enabled_ = false;
        return false;
    }
    if (test_ptr) {
        cudaFree(test_ptr);
        test_ptr = nullptr;
    }

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
    MakeContextCurrent();
    
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
    MakeContextCurrent();
    
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
        MakeContextCurrent();
        CUDA_CHECK(cudaStreamSynchronize(stream_));
    }
}

void CudaEngine::MakeContextCurrent() {
    if (!enabled_) {
        return;
    }
    CUDA_CHECK(cudaSetDevice(deviceId_));
}

void CudaEngine::UpdateArenaBatchFullPhysics(
    GpuBallState* ball,
    GpuCarState* cars,
    int numCars,
    float deltaTime,
    const GpuArenaCollisionData* arenaCollision
) {
    if (!enabled_) return;
    MakeContextCurrent();
    
    // Launch the complete GPU physics pipeline
    // This handles ALL physics and collisions without Bullet involvement
    LaunchGpuFullPhysicsStep(
        ball,
        1,  // Always 1 ball per arena
        cars,
        numCars,
        deltaTime,
        nullptr,  // collision grid - nullptr for now (using simplified collision)
        arenaCollision,  // Phase 2: Mesh collision data
        stream_
    );
    
    CUDA_CHECK(cudaGetLastError());
}

RS_NS_END

#endif // RS_CUDA_ENABLED
