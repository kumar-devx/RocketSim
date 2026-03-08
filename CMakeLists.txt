cmake_minimum_required (VERSION 3.18)
project("RocketSim" LANGUAGES CXX CUDA)

include(CTest)
option(ROCKETSIM_BUILD_TESTS "Build RocketSim test suite" ON)

# CUDA setup
find_package(CUDAToolkit REQUIRED)
message(STATUS "CUDA Version: ${CUDAToolkit_VERSION}")

# Set CUDA flags globally
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler")
add_compile_definitions(RS_CUDA_ENABLED)

# Gather source files
file(GLOB_RECURSE FILES_ROCKETSIM CONFIGURE_DEPENDS 
    "${PROJECT_SOURCE_DIR}/src/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/*.h"
)

file(GLOB_RECURSE FILES_CUDA CONFIGURE_DEPENDS 
    "${PROJECT_SOURCE_DIR}/src/Sim/Cuda/*.cu"
)

file(GLOB_RECURSE FILES_CUDA_HEADERS CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/src/Sim/Cuda/*.cuh"
)

file(GLOB_RECURSE FILES_BULLET CONFIGURE_DEPENDS 
    "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.cpp" 
    "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.h"
)

# Main library
add_library(RocketSim ${FILES_ROCKETSIM} ${FILES_BULLET} ${FILES_CUDA} ${FILES_CUDA_HEADERS})

# Set C++ standard
set_target_properties(RocketSim PROPERTIES 
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# CUDA settings for main library
set_target_properties(RocketSim PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_STANDARD 17
    CUDA_ARCHITECTURES "120"  # RTX 5070
)

target_include_directories(RocketSim PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
target_link_libraries(RocketSim PRIVATE CUDA::cudart CUDA::cuda_driver)

# CUDA compile options for main library
target_compile_options(RocketSim PRIVATE 
    $<$<COMPILE_LANGUAGE:CUDA>:
        -arch=sm_120
        --use_fast_math
        --expt-relaxed-constexpr
        -lineinfo
        -allow-unsupported-compiler
    >
)

# Build tests if enabled
if(BUILD_TESTING AND ROCKETSIM_BUILD_TESTS)
    # Create test executable
    add_executable(RocketSimGPUTests
        ${PROJECT_SOURCE_DIR}/tests/RocketSimGPUTests.cpp
    )
    
    # Include directories for tests
    target_include_directories(RocketSimGPUTests PRIVATE 
        ${PROJECT_SOURCE_DIR}/src
        ${CUDAToolkit_INCLUDE_DIRS}
    )
    
    # CRITICAL: Set CUDA properties for the test executable too!
    set_target_properties(RocketSimGPUTests PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CUDA_STANDARD 17
        CUDA_ARCHITECTURES "120"  # Same architecture as main library
    )
    
    # Link CUDA libraries
    target_link_libraries(RocketSimGPUTests PRIVATE 
        RocketSim
        CUDA::cudart
        CUDA::cuda_driver
    )
    
    # CRITICAL: Add CUDA compile options for the test executable
    # Even though it's a .cpp file, it includes CUDA headers and makes CUDA API calls
    target_compile_options(RocketSimGPUTests PRIVATE 
        $<$<COMPILE_LANGUAGE:CUDA>:
            -arch=sm_120
            -allow-unsupported-compiler
        >
    )
    
    add_test(NAME RocketSim.GPU.All COMMAND RocketSimGPUTests)
endif()