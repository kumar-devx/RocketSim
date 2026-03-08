#ifdef RS_CUDA_ENABLED

#include "CudaEngine.h"
#include <iostream>

RS_NS_START

// Simple test kernel
CUDA_KERNEL void TestKernel(float* output, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        output[idx] = idx * 2.0f;
    }
}

// Internal test function to verify CUDA is working
bool RunCudaSelfTest() {
    std::cout << "\n=== Testing CUDA Setup ===" << std::endl;
    
    if (!CudaMemoryManager::IsCudaAvailable()) {
        std::cout << "CUDA not available on this system" << std::endl;
        return false;
    }
    
    const int N = 1024;
    GpuBuffer<float> gpuData(N, true); // Unified memory
    
    // Launch test kernel
    int threadsPerBlock = 256;
    int numBlocks = (N + threadsPerBlock - 1) / threadsPerBlock;
    TestKernel<<<numBlocks, threadsPerBlock>>>(gpuData.get(), N);
    CUDA_CHECK(cudaGetLastError());
    
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Verify results
    bool success = true;
    for (int i = 0; i < 10; i++) {
        float expected = i * 2.0f;
        float actual = gpuData.get()[i];
        if (actual != expected) {
            std::cout << "Test FAILED at index " << i << ": expected " << expected << ", got " << actual << std::endl;
            success = false;
            break;
        }
    }
    
    if (success) {
        std::cout << "CUDA test PASSED! ?" << std::endl;
    }
    
    std::cout << "=========================" << std::endl;
    return success;
}

RS_NS_END

#endif // RS_CUDA_ENABLED
