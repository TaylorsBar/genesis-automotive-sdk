# Genesis Automotive SDK
# CMake Build Configuration
#
# Created by KC Speed Lab / CartelWorx
# Copyright © 2025 TaylorsBar. MIT Licensed.

cmake_minimum_required(VERSION 3.15)
project(GenesisAutomotiveSDK VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Source files
set(GENESIS_CORE_SOURCES
    Core/Fusion/GenesisEKF_Ultimate.cpp
)

set(GENESIS_CORE_HEADERS
    Core/Fusion/GenesisEKF_Ultimate.hpp
)

# Core library
add_library(genesis_core ${GENESIS_CORE_SOURCES})
target_include_directories(genesis_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Core
)
target_link_libraries(genesis_core
    PUBLIC
        Eigen3::Eigen
)

# iOS Platform Bridge (Objective-C++)
if(APPLE AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(GENESIS_IOS_SOURCES
        Core/Fusion/Platform/iOS/Bridge/GenesisCoreBridge.mm
    )
    
    add_library(genesis_ios ${GENESIS_IOS_SOURCES})
    target_link_libraries(genesis_ios
        PUBLIC
            genesis_core
    )
    target_compile_options(genesis_ios PRIVATE
        -fobjc-arc
    )
endif()

# Test executables (optional)
option(BUILD_TESTS "Build test executables" OFF)

if(BUILD_TESTS)
    # TODO: Add test framework (Google Test, Catch2, etc.)
    # add_executable(genesis_tests tests/test_ekf.cpp)
    # target_link_libraries(genesis_tests genesis_core)
endif()

# Examples (optional)
option(BUILD_EXAMPLES "Build example programs" OFF)

if(BUILD_EXAMPLES)
    # TODO: Add example applications
    # add_executable(example_fusion examples/fusion_demo.cpp)
    # target_link_libraries(example_fusion genesis_core)
endif()

# Installation
install(TARGETS genesis_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${GENESIS_CORE_HEADERS}
    DESTINATION include/Genesis/Core/Fusion
)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/GenesisAutomotiveSDKConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/GenesisAutomotiveSDKConfigVersion.cmake"
    DESTINATION lib/cmake/GenesisAutomotiveSDK
)

# Print configuration
message(STATUS "Genesis Automotive SDK Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")

# NOTE: This is a streamlined reference implementation.
# For complete production build configuration with cross-compilation support,
# additional platform targets, and optimization flags, see IMPLEMENTATION_GUIDE.md.
