#pragma once

#include "GpuTypes.h"
#include "MemoryManager.h"
#include "BallKernels.h"
#include "CarKernels.h"
#include "CollisionKernels.h"
#include <vector>
#include <mutex>

RS_NS_START

// Forward declarations
struct GpuArenaCollisionData;

// Main CUDA engine for RocketSim
class CudaEngine {
public:
    CudaEngine();
    ~CudaEngine();
    
    // Initialize CUDA subsystem
    bool Initialize();
    
    // Check if CUDA is available and initialized
    bool IsEnabled() const { return enabled_; }
    
    // Single arena update (CPU-friendly API)
    void UpdateArena(
        GpuBallState* ball,
        GpuCarState* cars, int numCars,
        float deltaTime
    );
    
    // Batch arena update (MASSIVE SPEEDUP!)
    void UpdateArenaBatch(
        GpuBallState* balls,
        GpuCarState* cars,
        int* carCounts,
        int numArenas,
        float deltaTime
    );
    
    // Full GPU physics pipeline
    void UpdateArenaBatchFullPhysics(
        GpuBallState* ball,
        GpuCarState* cars,
        int numCars,
        float deltaTime,
        const GpuArenaCollisionData* arenaCollision = nullptr
    );
    
    // Synchronize GPU (wait for all operations to complete)
    void Synchronize();

    // Ensure the CUDA primary context for this device is current on the calling thread.
    void MakeContextCurrent();

    // Accessor for internal stream so all kernel launches share one sync domain.
    cudaStream_t GetStream() const { return stream_; }
    
    // Thread-safe allocate device memory (holds alloc_mutex during operation)
    template<typename T>
    T* AllocateDeviceSafe(size_t count) {
        std::lock_guard<std::recursive_mutex> lock(alloc_mutex_);
        MakeContextCurrent();  // Ensure context on this thread
        return memoryManager_.AllocateDevice<T>(count);
    }
    
    // Thread-safe free device memory (holds alloc_mutex during operation)
    template<typename T>
    void FreeDeviceSafe(T*& ptr) {
        if (ptr) {
            std::lock_guard<std::recursive_mutex> lock(alloc_mutex_);
            MakeContextCurrent();  // Ensure context on this thread
            memoryManager_.Free(ptr);
            ptr = nullptr;
        }
    }
    
    // Get memory manager (should only be used under lock from AllocateDeviceSafe/FreeDeviceSafe)
    CudaMemoryManager& GetMemoryManager() { return memoryManager_; }
    
private:
    bool enabled_;
    int deviceId_;
    CudaMemoryManager memoryManager_;
    mutable std::recursive_mutex alloc_mutex_;  // Protect concurrent GPU memory allocations
    
    // CUDA streams for async operations
    cudaStream_t stream_;
};

RS_NS_END
