# Third-party dependencies, pinned by exact archive + SHA-256.
#
# To build offline or against a local checkout, point FetchContent at it, e.g.
#   -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=C:/src/JoltPhysics
#
# When bumping a version: update URL + URL_HASH here and the table in docs/IMPLEMENTATION_PLAN.md §1.

include(FetchContent)

# ---- Jolt Physics v5.6.0 ----------------------------------------------------------------------------
# Option names verified against JoltPhysics/Build/CMakeLists.txt at this commit.
set(TARGET_UNIT_TESTS OFF)
set(TARGET_HELLO_WORLD OFF)
set(TARGET_PERFORMANCE_TEST OFF)
set(TARGET_SAMPLES OFF)
set(TARGET_VIEWER OFF)
set(ENABLE_ALL_WARNINGS OFF)
set(ENABLE_INSTALL OFF)
set(JPH_BUILD_SHARED_LIBS OFF)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY ON)            # decision D3
set(DOUBLE_PRECISION OFF)                          # field is ~16.5 m; single precision is plenty
set(CROSS_PLATFORM_DETERMINISTIC ${FRCSIM_DETERMINISTIC})
set(INTERPROCEDURAL_OPTIMIZATION ON)
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF)         # never trap FP exceptions inside a JVM process
set(PROFILER_IN_DEBUG_AND_RELEASE OFF)             # decision D8
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF)       # decision D8
# GPU/compute backends (new in 5.6) are not used.
set(JPH_USE_DX12 OFF)
set(JPH_USE_VK OFF)
set(JPH_USE_MTL OFF)
set(JPH_USE_CPU_COMPUTE OFF)
# SIMD baseline: SSE4.2 only (decision D8). Low-end school laptops lack AVX2/F16C/FMA.
set(USE_SSE4_1 ON)
set(USE_SSE4_2 ON)
set(USE_AVX OFF)
set(USE_AVX2 OFF)
set(USE_AVX512 OFF)
set(USE_LZCNT OFF)
set(USE_TZCNT OFF)
set(USE_F16C OFF)
set(USE_FMADD OFF)

FetchContent_Declare(JoltPhysics
    URL https://github.com/jrouwe/JoltPhysics/archive/e77f175595e64cb44218cc9d9d56fc365ad0e36a.tar.gz
    URL_HASH SHA256=1f32328fb763135de10a244568d6ccb2ed9b1e6593fafe6dc6db5b2719d330bd
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR Build
    SYSTEM
)
FetchContent_MakeAvailable(JoltPhysics)

# ---- GoogleTest v1.18.0 -----------------------------------------------------------------------------
if(FRCSIM_BUILD_TESTS)
    set(INSTALL_GTEST OFF)
    set(BUILD_GMOCK OFF)
    set(gtest_force_shared_crt OFF) # static CRT, matching decision D3

    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
        URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(googletest)
endif()

# ---- nlohmann/json v3.12.0 (field loader, decision D6) ----------------------------------------------
set(JSON_BuildTests OFF)
set(JSON_Install OFF)

FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
FetchContent_MakeAvailable(nlohmann_json)

# Google Benchmark v1.9.5 is pinned for future micro-benchmarks but not fetched yet (decision D20):
#   URL https://github.com/google/benchmark/archive/refs/tags/v1.9.5.tar.gz
#   URL_HASH SHA256=9631341c82bac4a288bef951f8b26b41f69021794184ece969f8473977eaa340
