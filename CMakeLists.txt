cmake_minimum_required (VERSION 3.8)

project("RocketSim" LANGUAGES CXX)

include(CTest)
option(ROCKETSIM_BUILD_TESTS "Build RocketSim test suite" ON)

# Option to enable CUDA acceleration
option(ROCKETSIM_CUDA "Enable CUDA GPU acceleration" ON)

if(ROCKETSIM_CUDA)
    # IMPORTANT: VS 2025 is not officially supported by CUDA 13.1
    # Setting this flag to allow compilation (use at your own risk)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler")

    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    message(STATUS "CUDA enabled for RocketSim - GPU acceleration ON")
    message(STATUS "CUDA Version: ${CUDAToolkit_VERSION}")
    message(WARNING "VS 2025 detected with CUDA 13.1 - using -allow-unsupported-compiler flag")
    add_compile_definitions(RS_CUDA_ENABLED)
else()
    message(STATUS "CUDA disabled - CPU-only mode (use -DROCKETSIM_CUDA=ON to enable)")
endif()

# Add all headers and code files
file(GLOB_RECURSE FILES_ROCKETSIM CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp" "${PROJECT_SOURCE_DIR}/src/*.h")

# Add CUDA files if enabled
if(ROCKETSIM_CUDA)
    file(GLOB_RECURSE FILES_CUDA CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/Sim/Cuda/*.cu" "${PROJECT_SOURCE_DIR}/src/Sim/Cuda/*.cuh")
    set_source_files_properties(${FILES_CUDA} PROPERTIES LANGUAGE CUDA)
endif()

# Only include bullet headers when using MSVC, otherwise just code files
file(GLOB_RECURSE FILES_BULLET CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.cpp" "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.h")

if(ROCKETSIM_CUDA)
    add_library(RocketSim ${FILES_ROCKETSIM} ${FILES_BULLET} ${FILES_CUDA})
    target_compile_definitions(RocketSim PRIVATE RS_CUDA_ENABLED)
else()
    add_library(RocketSim ${FILES_ROCKETSIM} ${FILES_BULLET})
endif()

set_target_properties(RocketSim PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RocketSim PROPERTIES CXX_STANDARD 20)

# CUDA-specific settings
if(ROCKETSIM_CUDA)
    # Target GPU architectures
    # Keep broad native coverage and include PTX for forward-compat JIT on newer GPUs.
    # 75: Turing (RTX 2000)
    # 80: Ampere (A100, RTX 3000)
    # 86: Ampere (RTX 3090)
    # 89: Ada Lovelace (RTX 4000)
    # 90: Hopper
    set(ROCKETSIM_CUDA_ARCHS "75;80;86;89;90;90-virtual")

    # Blackwell/RTX 50-series support (compute capability 12.0) when toolkit supports it.
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL 12.8)
        list(APPEND ROCKETSIM_CUDA_ARCHS 120)
    endif()

    set_target_properties(RocketSim PROPERTIES 
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_STANDARD 17
        CUDA_ARCHITECTURES "${ROCKETSIM_CUDA_ARCHS}"
    )

    target_include_directories(RocketSim PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    target_link_libraries(RocketSim PRIVATE CUDA::cudart CUDA::cuda_driver)

    # Optimization flags for CUDA
    target_compile_options(RocketSim PRIVATE 
        $<$<COMPILE_LANGUAGE:CUDA>:
            --use_fast_math
            --expt-relaxed-constexpr
            -lineinfo
            -allow-unsupported-compiler
        >
    )
endif()

if(BUILD_TESTING AND ROCKETSIM_BUILD_TESTS)
    add_executable(RocketSimGPUTests
        ${PROJECT_SOURCE_DIR}/tests/RocketSimGPUTests.cpp
    )
    target_include_directories(RocketSimGPUTests PRIVATE ${PROJECT_SOURCE_DIR}/src)
    if(ROCKETSIM_CUDA)
        target_include_directories(RocketSimGPUTests PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    endif()
    target_link_libraries(RocketSimGPUTests PRIVATE RocketSim)
    set_target_properties(RocketSimGPUTests PROPERTIES CXX_STANDARD 20)
    add_test(NAME RocketSim.GPU.All COMMAND RocketSimGPUTests)
endif()