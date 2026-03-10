#pragma once

// Prefer the real CUDA runtime whenever it is available.
#if __has_include(<cuda_runtime.h>)
#include <cuda_runtime.h>

// IntelliSense fallback when CUDA headers are not configured in includePath.
#elif defined(__INTELLISENSE__)

#include <cstddef>
#include <cstdint>

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#ifndef __global__
#define __global__
#endif

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

struct cudaDeviceProp {
    char name[256];
    size_t totalGlobalMem;
    int maxThreadsPerBlock;
    int multiProcessorCount;
    int major;
    int minor;
    int clockRate;
};

inline const char* cudaGetErrorString(cudaError_t) {
    return "cuda_runtime.h unavailable (IntelliSense stub)";
}

extern "C" cudaError_t cudaMalloc(void** devPtr, size_t size);
extern "C" cudaError_t cudaMallocManaged(void** devPtr, size_t size);
extern "C" cudaError_t cudaFree(void* devPtr);
extern "C" cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind);
extern "C" cudaError_t cudaDeviceSynchronize(void);
extern "C" cudaError_t cudaGetDeviceCount(int* count);
extern "C" cudaError_t cudaSetDevice(int device);
extern "C" cudaError_t cudaGetDeviceProperties(cudaDeviceProp* prop, int device);
extern "C" cudaError_t cudaRuntimeGetVersion(int* runtimeVersion);
extern "C" cudaError_t cudaDriverGetVersion(int* driverVersion);

#else
#include <cstddef>
#include <cstdint>

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#ifndef __global__
#define __global__
#endif

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

struct cudaDeviceProp {
    char name[256];
    size_t totalGlobalMem;
    int maxThreadsPerBlock;
    int multiProcessorCount;
    int major;
    int minor;
    int clockRate;
};

inline const char* cudaGetErrorString(cudaError_t) {
    return "cuda_runtime.h unavailable (fallback stub)";
}

extern "C" cudaError_t cudaMalloc(void** devPtr, size_t size);
extern "C" cudaError_t cudaMallocManaged(void** devPtr, size_t size);
extern "C" cudaError_t cudaFree(void* devPtr);
extern "C" cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind);
extern "C" cudaError_t cudaDeviceSynchronize(void);
extern "C" cudaError_t cudaGetDeviceCount(int* count);
extern "C" cudaError_t cudaSetDevice(int device);
extern "C" cudaError_t cudaGetDeviceProperties(cudaDeviceProp* prop, int device);
extern "C" cudaError_t cudaRuntimeGetVersion(int* runtimeVersion);
extern "C" cudaError_t cudaDriverGetVersion(int* driverVersion);
#endif
