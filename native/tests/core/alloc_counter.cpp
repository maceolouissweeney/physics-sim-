// Replaces global operator new/delete for frcsim_core_tests so tests can assert that code paths do not
// allocate. Counting is only active while an AllocationCounter exists.

#include "core/alloc_counter.h"

#include <atomic>
#include <cstdlib>
#include <new>

namespace {

std::atomic<int> g_counting{0};
std::atomic<std::uint64_t> g_newCalls{0};

void* allocate(std::size_t size) {
    if (g_counting.load(std::memory_order_relaxed) > 0) {
        g_newCalls.fetch_add(1, std::memory_order_relaxed);
    }
    return std::malloc(size == 0 ? 1 : size);
}

void* allocateAligned(std::size_t size, std::align_val_t alignment) {
    if (g_counting.load(std::memory_order_relaxed) > 0) {
        g_newCalls.fetch_add(1, std::memory_order_relaxed);
    }
    const auto align = static_cast<std::size_t>(alignment);
#ifdef _WIN32
    return _aligned_malloc(size == 0 ? 1 : size, align);
#else
    const std::size_t rounded = ((size == 0 ? 1 : size) + align - 1) / align * align;
    return std::aligned_alloc(align, rounded);
#endif
}

void freeAligned(void* block) noexcept {
#ifdef _WIN32
    _aligned_free(block);
#else
    std::free(block);
#endif
}

} // namespace

namespace frcsim::test {

AllocationCounter::AllocationCounter() : m_start(g_newCalls.load()) {
    g_counting.fetch_add(1);
}

AllocationCounter::~AllocationCounter() {
    g_counting.fetch_sub(1);
}

std::uint64_t AllocationCounter::count() const {
    return g_newCalls.load() - m_start;
}

} // namespace frcsim::test

void* operator new(std::size_t size) {
    if (void* p = allocate(size)) {
        return p;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    if (void* p = allocate(size)) {
        return p;
    }
    throw std::bad_alloc();
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return allocate(size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    return allocate(size);
}

void* operator new(std::size_t size, std::align_val_t alignment) {
    if (void* p = allocateAligned(size, alignment)) {
        return p;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size, std::align_val_t alignment) {
    if (void* p = allocateAligned(size, alignment)) {
        return p;
    }
    throw std::bad_alloc();
}

void operator delete(void* block) noexcept {
    std::free(block);
}

void operator delete[](void* block) noexcept {
    std::free(block);
}

void operator delete(void* block, std::size_t) noexcept {
    std::free(block);
}

void operator delete[](void* block, std::size_t) noexcept {
    std::free(block);
}

void operator delete(void* block, std::align_val_t) noexcept {
    freeAligned(block);
}

void operator delete[](void* block, std::align_val_t) noexcept {
    freeAligned(block);
}

void operator delete(void* block, std::size_t, std::align_val_t) noexcept {
    freeAligned(block);
}

void operator delete[](void* block, std::size_t, std::align_val_t) noexcept {
    freeAligned(block);
}
