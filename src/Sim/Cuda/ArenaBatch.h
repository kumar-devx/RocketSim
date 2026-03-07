#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"
#include "MemoryManager.h"
#include <vector>

RS_NS_START

// Forward declaration
class Arena;

// Batch manager for simulating multiple arenas in parallel on GPU
// THIS IS WHERE WE GET 100x SPEEDUP!
class ArenaBatch {
public:
    ArenaBatch();
    ~ArenaBatch();
    
    // Add an arena to the batch
    void AddArena(Arena* arena);
    
    // Remove an arena from the batch
    void RemoveArena(Arena* arena);
    
    // Step all arenas in parallel on GPU
    // This is the key performance feature!
    void StepAll(int ticksToSimulate = 1);
    
    // Get number of arenas in batch
    int GetNumArenas() const { return static_cast<int>(arenas_.size()); }
    
    // Clear all arenas
    void Clear();
    
private:
    std::vector<Arena*> arenas_;
    
    // GPU buffers for batch processing
    GpuBuffer<GpuBallState> gpuBalls_;
    GpuBuffer<GpuCarState> gpuCars_;
    GpuBuffer<int> carOffsets_;  // Starting index of each arena's cars
    
    int totalCarsCapacity_;
    bool needsReallocation_;
    
    void AllocateGPUBuffers();
    void SyncToGPU();
    void SyncFromGPU();
};

RS_NS_END

#endif // RS_CUDA_ENABLED
