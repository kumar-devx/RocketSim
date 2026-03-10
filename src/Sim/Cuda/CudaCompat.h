#pragma once

// Prefer the real CUDA runtime whenever it is available.
#if __has_include(<cuda_runtime.h>)
#include <cuda_runtime.h>

// IntelliSense fallback when CUDA headers are not configured in includePath.
#elif defined(__INTELLISENSE__)

#include <cstddef>

typedef void* cudaStream_t;
typedef int cudaError_t;

enum cudaMemcpyKind {
    cudaMemcpyHostToHost = 0,
    cudaMemcpyHostToDevice = 1,
    cudaMemcpyDeviceToHost = 2,
    cudaMemcpyDeviceToDevice = 3,
    cudaMemcpyDefault = 4
};

static constexpr cudaError_t cudaSuccess = 0;

inline const char* cudaGetErrorString(cudaError_t) {
    return "cuda_runtime.h unavailable (IntelliSense stub)";
}

extern "C" cudaError_t cudaMalloc(void** devPtr, size_t size);
extern "C" cudaError_t cudaMallocManaged(void** devPtr, size_t size);
extern "C" cudaError_t cudaFree(void* devPtr);
extern "C" cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind);
extern "C" cudaError_t cudaDeviceSynchronize(void);

#else
#error "CUDA runtime headers were not found. Install CUDA Toolkit and/or configure include paths."
#endif
