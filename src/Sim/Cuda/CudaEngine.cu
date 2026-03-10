#ifdef RS_CUDA_ENABLED

#include "CudaEngine.h"
#include "GpuPhysicsPipeline.h"
#include <iostream>

RS_NS_START

CudaEngine::CudaEngine() : enabled_(false), deviceId_(0) {
    for (int i = 0; i < NUM_STREAMS; i++) streams_[i] = 0;
}

CudaEngine::~CudaEngine() {
    for (int i = 0; i < NUM_STREAMS; i++) {
        if (streams_[i]) {
            cudaStreamDestroy(streams_[i]);
            streams_[i] = nullptr;
        }
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

    // Create the pool of independent CUDA streams
    for (int i = 0; i < NUM_STREAMS; i++) {
        cudaError_t err = cudaStreamCreate(&streams_[i]);
        if (err != cudaSuccess) {
            std::cerr << "FATAL: Failed to create CUDA stream " << i << ": "
                      << cudaGetErrorString(err) << std::endl;
            // Destroy any already-created streams before returning
            for (int j = 0; j < i; j++) { cudaStreamDestroy(streams_[j]); streams_[j] = nullptr; }
            enabled_ = false;
            return false;
        }
    }

    // Test GPU memory allocation (MANDATORY)
    void* test_ptr = nullptr;
    cudaError_t err = cudaMalloc(&test_ptr, 1024 * 1024); // 1MB test
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
    std::cout << "CUDA initialized successfully! (" << NUM_STREAMS << " streams)" << std::endl;
    std::cout << "GPU-ONLY MODE: CPU fallback disabled" << std::endl;
    return true;
}

cudaStream_t CudaEngine::GetStreamForArena(const void* arenaPtr) const {
    // Mix pointer bits for better distribution across the pool
    size_t h = reinterpret_cast<size_t>(arenaPtr);
    h ^= h >> 16;
    return streams_[h % NUM_STREAMS];
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
        for (int i = 0; i < NUM_STREAMS; i++) {
            if (streams_[i]) CUDA_CHECK(cudaStreamSynchronize(streams_[i]));
        }
    }
}

void CudaEngine::SynchronizeStream(cudaStream_t stream) {
    if (enabled_ && stream) {
        MakeContextCurrent();
        CUDA_CHECK(cudaStreamSynchronize(stream));
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
    const GpuArenaCollisionData* arenaCollision,
    cudaStream_t stream
) {
    if (!enabled_) return;
    MakeContextCurrent();

    cudaStream_t activeStream = stream ? stream : streams_[0];
    LaunchGpuFullPhysicsStep(
        ball,
        1,
        cars,
        numCars,
        deltaTime,
        nullptr,
        arenaCollision,
        activeStream
    );
    CUDA_CHECK(cudaGetLastError());
}

void CudaEngine::UpdateArenaMultiTick(
    GpuBallState* ball,
    GpuCarState* cars,
    int numCars,
    const GpuCarControls* d_actionsAllTicks,
    int numTicks,
    float deltaTime,
    const GpuArenaCollisionData* arenaCollision,
    cudaStream_t stream
) {
    if (!enabled_) return;
    MakeContextCurrent();

    for (int t = 0; t < numTicks; t++) {
        // Write this tick's controls into car state (one thread per car, no sync needed).
        if (numCars > 0 && d_actionsAllTicks) {
            LaunchApplyControlsKernel(
                cars, numCars,
                d_actionsAllTicks + static_cast<ptrdiff_t>(t) * numCars,
                stream
            );
        }

        // Full physics step: ball + car integration, all collisions.
        // No cudaStreamSynchronize between ticks — same-stream ordering guarantees
        // that each kernel sees the results of the previous one.
        LaunchGpuFullPhysicsStep(
            ball, 1,
            cars, numCars,
            deltaTime,
            nullptr,
            arenaCollision,
            stream
        );
        CUDA_CHECK(cudaGetLastError());
    }
}

RS_NS_END

#endif // RS_CUDA_ENABLED
