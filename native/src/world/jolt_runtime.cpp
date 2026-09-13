#include "world/jolt_runtime.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

namespace frcsim {
namespace {

std::mutex g_runtimeMutex;
int g_runtimeRefCount = 0;
std::atomic<std::uint64_t> g_joltAllocations{0};

void traceToStderr(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fputc('\n', stderr);
}

#ifdef JPH_ENABLE_ASSERTS
bool reportJoltAssert(const char* expression, const char* message, const char* file, JPH::uint line) {
    std::fprintf(stderr, "[frcsim] Jolt assertion failed: %s (%s) at %s:%u\n", expression,
                 message != nullptr ? message : "", file, line);
    // Returning true breaks into the debugger. Asserts only exist in debug builds.
    return true;
}
#endif

#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
JPH::AllocateFunction g_baseAllocate = nullptr;
JPH::ReallocateFunction g_baseReallocate = nullptr;
JPH::AlignedAllocateFunction g_baseAlignedAllocate = nullptr;

void* countingAllocate(size_t size) {
    g_joltAllocations.fetch_add(1, std::memory_order_relaxed);
    return g_baseAllocate(size);
}

void* countingReallocate(void* block, size_t oldSize, size_t newSize) {
    g_joltAllocations.fetch_add(1, std::memory_order_relaxed);
    return g_baseReallocate(block, oldSize, newSize);
}

void* countingAlignedAllocate(size_t size, size_t alignment) {
    g_joltAllocations.fetch_add(1, std::memory_order_relaxed);
    return g_baseAlignedAllocate(size, alignment);
}

void installCountingAllocator() {
    JPH::RegisterDefaultAllocator();
    g_baseAllocate = JPH::Allocate;
    g_baseReallocate = JPH::Reallocate;
    g_baseAlignedAllocate = JPH::AlignedAllocate;
    JPH::Allocate = countingAllocate;
    JPH::Reallocate = countingReallocate;
    JPH::AlignedAllocate = countingAlignedAllocate;
}
#else
void installCountingAllocator() {
    JPH::RegisterDefaultAllocator();
}
#endif

} // namespace

JoltRuntime::JoltRuntime() {
    std::lock_guard lock(g_runtimeMutex);
    if (g_runtimeRefCount++ == 0) {
        installCountingAllocator();
        JPH::Trace = traceToStderr;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = reportJoltAssert;)
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

JoltRuntime::~JoltRuntime() {
    std::lock_guard lock(g_runtimeMutex);
    if (--g_runtimeRefCount == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

std::uint64_t joltAllocationCount() noexcept {
    return g_joltAllocations.load(std::memory_order_relaxed);
}

} // namespace frcsim
