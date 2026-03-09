cmake_minimum_required(VERSION 3.20)
project(RocketSim VERSION 1.0.0 LANGUAGES CXX CUDA)

# Find CUDA
find_package(CUDAToolkit REQUIRED)

message(STATUS "CUDA Version: ${CUDAToolkit_VERSION}")

# Set CUDA architectures for RTX 5070
set(CMAKE_CUDA_ARCHITECTURES "120")

# Gather source files
file(GLOB_RECURSE ROCKETSIM_SOURCES
    CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
    "src/Sim/Cuda/*.cu"
    "src/Sim/Cuda/*.cuh"
)

file(GLOB_RECURSE BULLET_SOURCES
    CONFIGURE_DEPENDS
    "libsrc/bullet3-3.24/*.cpp"
    "libsrc/bullet3-3.24/*.h"
)

# Create SHARED library (DLL) instead of STATIC
add_library(RocketSim SHARED
    ${ROCKETSIM_SOURCES}
    ${BULLET_SOURCES}
)

# Set properties
set_target_properties(RocketSim PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(RocketSim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    PRIVATE
        ${CUDAToolkit_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/libsrc/bullet3-3.24/src
)

# CUDA compile options
target_compile_options(RocketSim PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        -arch=sm_120
        --use_fast_math
        --expt-relaxed-constexpr
        --expt-extended-lambda
        -allow-unsupported-compiler
    >
)

# Link CUDA libraries
target_link_libraries(RocketSim PRIVATE
    CUDA::cudart
    CUDA::cuda_driver
)

# Windows-specific settings
if(WIN32)
    # Fix for NVCC PDB flag issue - separate the flags
    target_compile_options(RocketSim PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>: /EHsc /MP>
        $<$<COMPILE_LANGUAGE:CUDA>: -Xcompiler="/EHsc" -Xcompiler="/MP">
    )
    
    # Handle PDB generation properly for CUDA
    if(MSVC)
        # Get the PDB name from target properties
        get_target_property(ROCKETSIM_PDB_NAME RocketSim PDB_NAME)
        if(NOT ROCKETSIM_PDB_NAME)
            set(ROCKETSIM_PDB_NAME "${PROJECT_NAME}")
        endif()
        
        # Set PDB directory and name
        set_target_properties(RocketSim PROPERTIES
            PDB_NAME "${ROCKETSIM_PDB_NAME}"
            PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            COMPILE_PDB_NAME "${ROCKETSIM_PDB_NAME}"
            COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        )
        
        # Add separate PDB flags for CUDA files (FIXED - no comma)
        target_compile_options(RocketSim PRIVATE
            $<$<COMPILE_LANGUAGE:CUDA>:
                -Xcompiler=-Fd${CMAKE_CURRENT_BINARY_DIR}/${ROCKETSIM_PDB_NAME}.pdb
                -Xcompiler=-FS
            >
        )
    endif()
endif()

# Define RocketSim export macro
target_compile_definitions(RocketSim PRIVATE
    RocketSim_EXPORTS
    $<$<COMPILE_LANGUAGE:CUDA>:RS_CUDA_ENABLED>
)

# Generate compile_commands.json for better IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)