cmake_minimum_required(VERSION 3.25)

# Single source of truth for the version, shared with the Gradle build.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/../VERSION" FRCSIM_VERSION LIMIT_COUNT 1)

project(frcsim VERSION ${FRCSIM_VERSION} LANGUAGES C CXX)

# ---- Options ----------------------------------------------------------------------------------------
option(FRCSIM_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(FRCSIM_BUILD_JNI "Compile the JNI bindings into the shared library (requires a JDK)" ON)
option(FRCSIM_BUILD_BENCH "Build benchmarks and the scenario runner" OFF)
option(FRCSIM_WARNINGS_AS_ERRORS "Treat warnings in frcsim code as errors" OFF)
option(FRCSIM_DETERMINISTIC "Build Jolt with cross-platform determinism (slower)" OFF)
set(FRCSIM_SANITIZE "" CACHE STRING "Comma-separated sanitizers for GCC/Clang, e.g. address,undefined")

# ---- Global settings (must precede dependencies) ----------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# Static MSVC runtime everywhere (decision D3): self-contained DLL, matches Jolt's default.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Let our normal variables override option() defaults inside dependency projects.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# Keep executables and the shared library side by side so tests find the DLL on Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FrcsimPlatform)
include(FrcsimCompilerOptions)
include(FrcsimDependencies)

# ---- Targets ----------------------------------------------------------------------------------------
add_subdirectory(src)

if(FRCSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(FRCSIM_BUILD_BENCH)
    add_subdirectory(bench)
endif()
