#pragma once

#ifdef RS_CUDA_ENABLED

#include "GpuTypes.h"
#include <vector>
#include <memory>

RS_NS_START

// Manages GPU memory allocation and transfers
class CudaMemoryManager {
public:
    CudaMemoryManager();
    ~CudaMemoryManager();
    
    // Allocate GPU memory
    template<typename T>
    T* AllocateDevice(size_t count) {
        T* ptr = nullptr;
        CUDA_CHECK(cudaMalloc(&ptr, count * sizeof(T)));
        return ptr;
    }
    
    // Allocate unified memory (accessible from CPU and GPU)
    template<typename T>
    T* AllocateUnified(size_t count) {
        T* ptr = nullptr;
        CUDA_CHECK(cudaMallocManaged(&ptr, count * sizeof(T)));
        return ptr;
    }
    
    // Free memory
    void Free(void* ptr) {
        if (ptr) {
            CUDA_CHECK(cudaFree(ptr));
        }
    }

    // Free memory and nullify pointer (safer)
    template<typename T>
    void Free(T*& ptr) {
        if (ptr) {
            CUDA_CHECK(cudaFree(ptr));
            ptr = nullptr;
        }
    }
    
    // Copy data to device
    template<typename T>
    void CopyToDevice(T* dst, const T* src, size_t count) {
        CUDA_CHECK(cudaMemcpy(dst, src, count * sizeof(T), cudaMemcpyHostToDevice));
    }
    
    // Copy data from device
    template<typename T>
    void CopyFromDevice(T* dst, const T* src, size_t count) {
        CUDA_CHECK(cudaMemcpy(dst, src, count * sizeof(T), cudaMemcpyDeviceToHost));
    }
    
    // Synchronize GPU
    void Synchronize() {
        CUDA_CHECK(cudaDeviceSynchronize());
    }
    
    // Check if CUDA is available
    static bool IsCudaAvailable();
    
    // Get device properties
    static void PrintDeviceInfo();
};

// RAII wrapper for GPU memory - SAFE VERSION
template<typename T>
class GpuBuffer {
public:
    GpuBuffer() : ptr_(nullptr), size_(0), valid_(true) {}

    explicit GpuBuffer(size_t size, bool useUnified = false) : size_(size), valid_(false) {
        try {
            if (useUnified) {
                if (CUDA_TRY(cudaMallocManaged(&ptr_, size * sizeof(T)))) {
                    valid_ = true;
                }
            } else {
                if (CUDA_TRY(cudaMalloc(&ptr_, size * sizeof(T)))) {
                    valid_ = true;
                }
            }

            if (!valid_) {
                fprintf(stderr, "GpuBuffer: Failed to allocate %zu bytes\n", size * sizeof(T));
                ptr_ = nullptr;
            }
        } catch (...) {
            ptr_ = nullptr;
            valid_ = false;
        }
    }

    ~GpuBuffer() {
        if (ptr_) {
            cudaFree(ptr_);
            ptr_ = nullptr;
        }
    }

    // No copy
    GpuBuffer(const GpuBuffer&) = delete;
    GpuBuffer& operator=(const GpuBuffer&) = delete;

    // Move semantics
    GpuBuffer(GpuBuffer&& other) noexcept 
        : ptr_(other.ptr_), size_(other.size_), valid_(other.valid_) {
        other.ptr_ = nullptr;
        other.size_ = 0;
        other.valid_ = false;
    }

    GpuBuffer& operator=(GpuBuffer&& other) noexcept {
        if (this != &other) {
            if (ptr_) {
                cudaFree(ptr_);
                ptr_ = nullptr;
            }
            ptr_ = other.ptr_;
            size_ = other.size_;
            valid_ = other.valid_;
            other.ptr_ = nullptr;
            other.size_ = 0;
            other.valid_ = false;
        }
        return *this;
    }

    T* get() { return ptr_; }
    const T* get() const { return ptr_; }
    size_t size() const { return size_; }
    bool isValid() const { return valid_ && ptr_ != nullptr; }

    void CopyToDevice(const T* src, size_t count) {
        if (!valid_ || !ptr_) return;
        CUDA_TRY(cudaMemcpy(ptr_, src, count * sizeof(T), cudaMemcpyHostToDevice));
    }

    void CopyFromDevice(T* dst, size_t count) const {
        if (!valid_ || !ptr_) return;
        CUDA_TRY(cudaMemcpy(dst, ptr_, count * sizeof(T), cudaMemcpyDeviceToHost));
    }

private:
    T* ptr_;
    size_t size_;
    bool valid_;  // Track if allocation succeeded
};

RS_NS_END

#endif // RS_CUDA_ENABLED
