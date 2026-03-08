#ifdef RS_CUDA_ENABLED

#include "MemoryManager.h"
#include <iostream>

RS_NS_START

CudaMemoryManager::CudaMemoryManager() {
    // Initialize CUDA
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    if (error != cudaSuccess || deviceCount == 0) {
        std::cerr << "No CUDA devices found!" << std::endl;
        return;
    }
    
    // Set device 0 as default
    CUDA_CHECK(cudaSetDevice(0));
}

CudaMemoryManager::~CudaMemoryManager() {
    // Do not call cudaDeviceReset() here.
    // Resetting the process-wide CUDA device during object teardown can race
    // with other static/global destructors and cause exit-time crashes.
}

bool CudaMemoryManager::IsCudaAvailable() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    return (error == cudaSuccess && deviceCount > 0);
}

void CudaMemoryManager::PrintDeviceInfo() {
    int deviceCount = 0;
    CUDA_CHECK(cudaGetDeviceCount(&deviceCount));

    int runtimeVersion = 0;
    int driverVersion = 0;
    cudaRuntimeGetVersion(&runtimeVersion);
    cudaDriverGetVersion(&driverVersion);

    auto printCudaVersion = [](const char* label, int version) {
        int major = version / 1000;
        int minor = (version % 1000) / 10;
        std::cout << "  " << label << ": " << major << "." << minor << " (" << version << ")" << std::endl;
    };
    
    std::cout << "=== CUDA Device Information ===" << std::endl;
    std::cout << "Number of CUDA devices: " << deviceCount << std::endl;
    printCudaVersion("CUDA Runtime Version", runtimeVersion);
    printCudaVersion("CUDA Driver Version", driverVersion);
    
    for (int i = 0; i < deviceCount; i++) {
        cudaDeviceProp prop;
        CUDA_CHECK(cudaGetDeviceProperties(&prop, i));
        
        std::cout << "\nDevice " << i << ": " << prop.name << std::endl;
        std::cout << "  Compute Capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "  Total Memory: " << (prop.totalGlobalMem / (1024 * 1024)) << " MB" << std::endl;
        std::cout << "  Max Threads Per Block: " << prop.maxThreadsPerBlock << std::endl;
        std::cout << "  Multiprocessors: " << prop.multiProcessorCount << std::endl;
        std::cout << "  Clock Rate: " << (prop.clockRate / 1000) << " MHz" << std::endl;
    }
    std::cout << "===============================" << std::endl;
}

RS_NS_END

#endif // RS_CUDA_ENABLED
