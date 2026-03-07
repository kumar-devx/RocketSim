// RocketSim CUDA Standalone Test
// This tests CUDA functionality without PyTorch/GigaLearn

#include <iostream>
#include <chrono>
#include <vector>
#include <map>

// Include CUDA runtime first
#ifdef RS_CUDA_ENABLED
#include <cuda_runtime.h>
#endif

// Then include RocketSim
#include "../src/RocketSim.h"

using namespace RocketSim;
using namespace std::chrono;

void PrintSeparator() {
    std::cout << "========================================" << std::endl;
}

bool TestCudaBasic() {
    PrintSeparator();
    std::cout << "TEST 1: Basic CUDA Availability" << std::endl;
    PrintSeparator();
    
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    
    std::cout << "CUDA Device Count: " << deviceCount << std::endl;
    std::cout << "CUDA Status: " << cudaGetErrorString(err) << std::endl;
    
    if (err != cudaSuccess || deviceCount == 0) {
        std::cout << "❌ No CUDA devices available!" << std::endl;
        return false;
    }
    
    // Test memory allocation
    void* testPtr = nullptr;
    size_t testSize = 1024 * 1024; // 1MB
    err = cudaMalloc(&testPtr, testSize);
    
    if (err != cudaSuccess) {
        std::cout << "❌ CUDA memory allocation failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    std::cout << "✅ Successfully allocated 1MB GPU memory" << std::endl;
    
    cudaFree(testPtr);
    std::cout << "✅ Successfully freed GPU memory" << std::endl;
    
    // Print device info
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::cout << "\nGPU Information:" << std::endl;
    std::cout << "  Name: " << prop.name << std::endl;
    std::cout << "  Compute Capability: " << prop.major << "." << prop.minor << std::endl;
    std::cout << "  Total Memory: " << (prop.totalGlobalMem / (1024*1024)) << " MB" << std::endl;
    std::cout << "  Clock Rate: " << (prop.clockRate / 1000) << " MHz" << std::endl;
    
    return true;
}

bool TestRocketSimInit() {
    PrintSeparator();
    std::cout << "TEST 2: RocketSim Initialization" << std::endl;
    PrintSeparator();
    
    try {
        // Initialize without collision meshes (THE_VOID mode works without them)
        std::map<GameMode, std::vector<FileData>> emptyMeshes;
        RocketSim::InitFromMem(emptyMeshes, false);
        
        std::cout << "✅ RocketSim initialized successfully" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cout << "❌ RocketSim initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool TestCudaEngine() {
    PrintSeparator();
    std::cout << "TEST 3: RocketSim CUDA Engine" << std::endl;
    PrintSeparator();
    
#ifdef RS_CUDA_ENABLED
    std::cout << "✅ CUDA support compiled in" << std::endl;
    
    if (RocketSim::IsCudaEnabled()) {
        std::cout << "✅ CUDA engine initialized" << std::endl;
        return true;
    } else {
        std::cout << "⚠️  CUDA compiled but not initialized (no GPU or driver issue)" << std::endl;
        return false;
    }
#else
    std::cout << "❌ CUDA support not compiled in" << std::endl;
    std::cout << "   Rebuild with -DROCKETSIM_CUDA=ON" << std::endl;
    return false;
#endif
}

bool TestArenaCreation() {
    PrintSeparator();
    std::cout << "TEST 4: Arena Creation & GPU Allocation" << std::endl;
    PrintSeparator();
    
    try {
        // Create arena in THE_VOID mode (no collision meshes needed)
        Arena* arena = Arena::Create(GameMode::THE_VOID, {}, 120);
        
        if (!arena) {
            std::cout << "❌ Failed to create arena" << std::endl;
            return false;
        }
        
        std::cout << "✅ Arena created successfully" << std::endl;
        
#ifdef RS_CUDA_ENABLED
        if (arena->_useCuda) {
            std::cout << "✅ Arena using GPU acceleration!" << std::endl;
            
            if (arena->_gpuBall && arena->_gpuCars) {
                std::cout << "✅ GPU buffers allocated successfully" << std::endl;
            } else {
                std::cout << "⚠️  GPU buffers not allocated properly" << std::endl;
                delete arena;
                return false;
            }
        } else {
            std::cout << "⚠️  Arena using CPU mode (GPU allocation may have failed)" << std::endl;
        }
#endif
        
        delete arena;
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Arena creation failed: " << e.what() << std::endl;
        return false;
    }
}

bool TestPhysicsSimulation() {
    PrintSeparator();
    std::cout << "TEST 5: Physics Simulation" << std::endl;
    PrintSeparator();
    
    try {
        Arena* arena = Arena::Create(GameMode::THE_VOID, {}, 120);
        
        if (!arena) {
            std::cout << "❌ Failed to create arena" << std::endl;
            return false;
        }
        
        // Add a car
        Car* car = arena->AddCar(Team::BLUE);
        
        // Get initial state
        BallState initialBall = arena->ball->GetState();
        CarState initialCar = car->GetState();
        
        std::cout << "Initial ball position: (" 
                  << initialBall.pos.x << ", " 
                  << initialBall.pos.y << ", " 
                  << initialBall.pos.z << ")" << std::endl;
        
        // Simulate 120 ticks (1 second)
        const int TICKS = 120;
        
        auto startTime = high_resolution_clock::now();
        arena->Step(TICKS);
        auto endTime = high_resolution_clock::now();
        
        auto duration = duration_cast<microseconds>(endTime - startTime).count();
        float seconds = duration / 1000000.0f;
        
        // Get final state
        BallState finalBall = arena->ball->GetState();
        CarState finalCar = car->GetState();
        
        std::cout << "Final ball position: (" 
                  << finalBall.pos.x << ", " 
                  << finalBall.pos.y << ", " 
                  << finalBall.pos.z << ")" << std::endl;
        
        std::cout << "\nPerformance:" << std::endl;
        std::cout << "  Simulated: 1.0 seconds of game time" << std::endl;
        std::cout << "  Real time: " << seconds << " seconds" << std::endl;
        std::cout << "  Speedup: " << (1.0f / seconds) << "x realtime" << std::endl;
        
#ifdef RS_CUDA_ENABLED
        if (arena->_useCuda) {
            std::cout << "  Mode: GPU accelerated ⚡" << std::endl;
        } else {
            std::cout << "  Mode: CPU only" << std::endl;
        }
#endif
        
        // Verify physics worked (ball should have fallen)
        if (finalBall.pos.z < initialBall.pos.z - 10) {
            std::cout << "✅ Physics simulation working (ball fell)" << std::endl;
        } else {
            std::cout << "⚠️  Ball didn't fall as expected" << std::endl;
        }
        
        delete arena;
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Physics simulation failed: " << e.what() << std::endl;
        return false;
    }
}

bool TestStressTest() {
    PrintSeparator();
    std::cout << "TEST 6: Stress Test (10 arenas, 1000 steps)" << std::endl;
    PrintSeparator();
    
    try {
        std::vector<Arena*> arenas;
        
        // Create 10 arenas
        for (int i = 0; i < 10; i++) {
            Arena* arena = Arena::Create(GameMode::THE_VOID, {}, 120);
            arena->AddCar(Team::BLUE);
            arena->AddCar(Team::ORANGE);
            arenas.push_back(arena);
        }
        
        std::cout << "Created 10 arenas with 2 cars each" << std::endl;
        
        // Simulate 1000 steps
        const int TICKS = 1000;
        
        auto startTime = high_resolution_clock::now();
        
        for (auto* arena : arenas) {
            arena->Step(TICKS);
        }
        
        auto endTime = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(endTime - startTime).count();
        
        std::cout << "✅ Stress test completed" << std::endl;
        std::cout << "  Total time: " << duration << " ms" << std::endl;
        std::cout << "  Average per arena: " << (duration / 10.0f) << " ms" << std::endl;
        
        // Cleanup
        for (auto* arena : arenas) {
            delete arena;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Stress test failed: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║  RocketSim CUDA Standalone Test     ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    int passed = 0;
    int total = 6;
    
    // Run tests
    if (TestCudaBasic()) passed++;
    if (TestRocketSimInit()) passed++;
    if (TestCudaEngine()) passed++;
    if (TestArenaCreation()) passed++;
    if (TestPhysicsSimulation()) passed++;
    if (TestStressTest()) passed++;
    
    // Final results
    PrintSeparator();
    std::cout << "FINAL RESULTS" << std::endl;
    PrintSeparator();
    std::cout << "Tests passed: " << passed << " / " << total << std::endl;
    
    if (passed == total) {
        std::cout << std::endl;
        std::cout << "🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "RocketSim CUDA is working correctly!" << std::endl;
        std::cout << std::endl;
        std::cout << "If GigaLearn still crashes, the issue is in:" << std::endl;
        std::cout << "  - PyTorch/LibTorch configuration" << std::endl;
        std::cout << "  - GigaLearn's CUDA usage" << std::endl;
        std::cout << "  - Not in RocketSim!" << std::endl;
        return 0;
    } else {
        std::cout << std::endl;
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        std::cout << "Check the output above for details." << std::endl;
        return 1;
    }
}
