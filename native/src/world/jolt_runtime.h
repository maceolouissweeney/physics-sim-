#pragma once

#include <cstdint>

namespace frcsim {

/**
 * RAII handle for Jolt's process-wide state (allocator hooks, type factory, type registry).
 *
 * Jolt requires one-time global registration before any physics object is created. Each World holds a
 * JoltRuntime so registration happens on first use and is undone when the last world is destroyed.
 * Thread-safe.
 */
class JoltRuntime {
public:
    JoltRuntime();
    ~JoltRuntime();

    JoltRuntime(const JoltRuntime&) = delete;
    JoltRuntime& operator=(const JoltRuntime&) = delete;
};

/**
 * Number of allocations (allocate, reallocate, aligned allocate) made through Jolt's allocator since
 * process start, across all worlds. Used by tests to verify stepping does not allocate.
 */
[[nodiscard]] std::uint64_t joltAllocationCount() noexcept;

} // namespace frcsim
