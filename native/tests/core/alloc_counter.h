#pragma once

#include <cstdint>

namespace frcsim::test {

/// Enables counting of global operator new calls (all threads) for this object's lifetime.
class AllocationCounter {
public:
    AllocationCounter();
    ~AllocationCounter();

    AllocationCounter(const AllocationCounter&) = delete;
    AllocationCounter& operator=(const AllocationCounter&) = delete;

    /// operator new calls since construction.
    [[nodiscard]] std::uint64_t count() const;

private:
    std::uint64_t m_start;
};

} // namespace frcsim::test
