#pragma once

#include <cuda_runtime.h>
#include <float.h>
#include <math.h>

#define RS_INLINE __host__ __device__ __forceinline__

constexpr float LARGE_FLOAT = 1e18f;
constexpr float SIMD_EPSILON = 1.192092896e-07f;
