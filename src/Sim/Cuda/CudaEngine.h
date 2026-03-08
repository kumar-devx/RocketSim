#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"
#include "MemoryManager.h"
#include "BallKernels.h"
#include "CarKernels.h"
#include "CollisionKernels.h"
#include <vector>

RS_NS_START

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
    
    // Synchronize GPU (wait for all operations to complete)
    void Synchronize();

    // Accessor for internal stream so all kernel launches share one sync domain.
    cudaStream_t GetStream() const { return stream_; }
    
    // Get memory manager
    CudaMemoryManager& GetMemoryManager() { return memoryManager_; }
    
private:
    bool enabled_;
    CudaMemoryManager memoryManager_;
    
    // CUDA streams for async operations
    cudaStream_t stream_;
};

RS_NS_END

#endif // RS_CUDA_ENABLED
