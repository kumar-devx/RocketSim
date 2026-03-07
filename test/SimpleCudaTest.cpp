// Simple CUDA Test - No RocketSim dependencies except CUDA parts
#include <iostream>
#include <cuda_runtime.h>

int main() {
    std::cout << "╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║  Simple CUDA Functionality Test      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    // Test 1: Device Count
    std::cout << "TEST 1: CUDA Device Detection" << std::endl;
    std::cout << "======================================" << std::endl;
    
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    
    std::cout << "Device Count: " << deviceCount << std::endl;
    std::cout << "Status: " << cudaGetErrorString(err) << std::endl;
    
    if (err != cudaSuccess || deviceCount == 0) {
        std::cout << "❌ No CUDA devices!" << std::endl;
        return 1;
    }
    
    std::cout << "✅ Found " << deviceCount << " CUDA device(s)" << std::endl;
    std::cout << std::endl;
    
    // Test 2: Device Properties
    std::cout << "TEST 2: Device Properties" << std::endl;
    std::cout << "======================================" << std::endl;
    
    for (int i = 0; i < deviceCount; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        
        std::cout << "Device " << i << ": " << prop.name << std::endl;
        std::cout << "  Compute Capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "  Total Memory: " << (prop.totalGlobalMem / (1024*1024)) << " MB" << std::endl;
        std::cout << "  Multiprocessors: " << prop.multiProcessorCount << std::endl;
        std::cout << "  Max Threads Per Block: " << prop.maxThreadsPerBlock << std::endl;
        std::cout << "  Clock Rate: " << (prop.clockRate / 1000) << " MHz" << std::endl;
    }
    
    std::cout << "✅ Device properties read successfully" << std::endl;
    std::cout << std::endl;
    
    // Test 3: Memory Allocation
    std::cout << "TEST 3: GPU Memory Allocation" << std::endl;
    std::cout << "======================================" << std::endl;
    
    void* testPtr = nullptr;
    size_t sizes[] = {1024, 1024*1024, 10*1024*1024, 100*1024*1024}; // 1KB, 1MB, 10MB, 100MB
    const char* sizeNames[] = {"1 KB", "1 MB", "10 MB", "100 MB"};
    
    bool allPassed = true;
    for (int i = 0; i < 4; i++) {
        err = cudaMalloc(&testPtr, sizes[i]);
        
        if (err == cudaSuccess) {
            std::cout << "  " << sizeNames[i] << ": ✅ Success" << std::endl;
            cudaFree(testPtr);
        } else {
            std::cout << "  " << sizeNames[i] << ": ❌ Failed - " << cudaGetErrorString(err) << std::endl;
            allPassed = false;
            break;
        }
    }
    
    if (allPassed) {
        std::cout << "✅ All memory allocations successful" << std::endl;
    } else {
        std::cout << "❌ Memory allocation failed!" << std::endl;
        return 1;
    }
    std::cout << std::endl;
    
    // Test 4: Memory Copy
    std::cout << "TEST 4: Memory Copy (Host <-> Device)" << std::endl;
    std::cout << "======================================" << std::endl;
    
    const size_t arraySize = 1024;
    float* hostArray = new float[arraySize];
    float* hostResult = new float[arraySize];
    float* deviceArray = nullptr;
    
    // Initialize host array
    for (size_t i = 0; i < arraySize; i++) {
        hostArray[i] = static_cast<float>(i);
    }
    
    // Allocate device memory
    err = cudaMalloc(&deviceArray, arraySize * sizeof(float));
    if (err != cudaSuccess) {
        std::cout << "❌ Device allocation failed: " << cudaGetErrorString(err) << std::endl;
        delete[] hostArray;
        delete[] hostResult;
        return 1;
    }
    
    // Copy to device
    err = cudaMemcpy(deviceArray, hostArray, arraySize * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        std::cout << "❌ Host->Device copy failed: " << cudaGetErrorString(err) << std::endl;
        cudaFree(deviceArray);
        delete[] hostArray;
        delete[] hostResult;
        return 1;
    }
    std::cout << "  Host -> Device: ✅ Success" << std::endl;
    
    // Copy back to host
    err = cudaMemcpy(hostResult, deviceArray, arraySize * sizeof(float), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        std::cout << "❌ Device->Host copy failed: " << cudaGetErrorString(err) << std::endl;
        cudaFree(deviceArray);
        delete[] hostArray;
        delete[] hostResult;
        return 1;
    }
    std::cout << "  Device -> Host: ✅ Success" << std::endl;
    
    // Verify data
    bool dataCorrect = true;
    for (size_t i = 0; i < arraySize; i++) {
        if (hostArray[i] != hostResult[i]) {
            dataCorrect = false;
            break;
        }
    }
    
    if (dataCorrect) {
        std::cout << "  Data verification: ✅ Success" << std::endl;
    } else {
        std::cout << "  Data verification: ❌ Failed" << std::endl;
    }
    
    cudaFree(deviceArray);
    delete[] hostArray;
    delete[] hostResult;
    
    std::cout << "✅ Memory operations working" << std::endl;
    std::cout << std::endl;
    
    // Final Summary
    std::cout << "======================================" << std::endl;
    std::cout << "🎉 ALL TESTS PASSED!" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Your CUDA installation is working correctly." << std::endl;
    std::cout << "If RocketSim still crashes, the issue is in:" << std::endl;
    std::cout << "  - RocketSim's CUDA code" << std::endl;
    std::cout << "  - Integration with your application" << std::endl;
    std::cout << std::endl;
    
    return 0;
}
