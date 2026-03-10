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
    
    // Full GPU physics pipeline (single tick, optional explicit stream).
    // stream == 0 means use streams_[0].
    void UpdateArenaBatchFullPhysics(
        GpuBallState* ball,
        GpuCarState* cars,
        int numCars,
        float deltaTime,
        const GpuArenaCollisionData* arenaCollision = nullptr,
        cudaStream_t stream = 0
    );

    // True GPU ownership API — run numTicks physics iterations on GPU with
    // a single H2D action upload.  No CPU sync between ticks; all kernels are
    // ordered on 'stream' by CUDA stream semantics.
    // d_actionsAllTicks layout: [tick0_car0 .. tick0_carN, tick1_car0 .. ]
    void UpdateArenaMultiTick(
        GpuBallState* ball,
        GpuCarState* cars,
        int numCars,
        const GpuCarControls* d_actionsAllTicks,
        int numTicks,
        float deltaTime,
        const GpuArenaCollisionData* arenaCollision,
        cudaStream_t stream
    );
    
    // Synchronize GPU — waits for ALL pooled streams to drain.
    void Synchronize();

    // Synchronize a single specific stream.
    void SynchronizeStream(cudaStream_t stream);

    // Ensure the CUDA primary context for this device is current on the calling thread.
    void MakeContextCurrent();

    // Returns the default stream (streams_[0]); used for non-arena GPU work.
    cudaStream_t GetStream() const { return streams_[0]; }

    // Returns the dedicated stream assigned to 'arenaPtr' (hash of pointer → pool).
    // Different arenas get different streams for true parallel GPU execution.
    cudaStream_t GetStreamForArena(const void* arenaPtr) const;
    
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
    // Number of independent CUDA streams in the pool.
    // Arenas are round-robin assigned so concurrent arenas overlap on GPU.
    static constexpr int NUM_STREAMS = 4;

    bool enabled_;
    int deviceId_;
    CudaMemoryManager memoryManager_;
    mutable std::recursive_mutex alloc_mutex_;  // Protect concurrent GPU memory allocations

    // Stream pool — each entry is an independent in-order CUDA stream.
    cudaStream_t streams_[NUM_STREAMS];
};

RS_NS_END
