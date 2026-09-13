#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

namespace frcsim {

/**
 * Fixed-capacity event buffer for Jolt callbacks.
 *
 * Producers (contact callbacks on physics job threads) may call tryPush() concurrently; it is lock-free
 * and never allocates. The consumer reads events() and calls clear() on the stepping thread only while
 * no producers are running (i.e. between PhysicsSystem::Update calls). Events beyond capacity are
 * dropped and counted.
 */
template <typename T>
class EventBuffer {
public:
    explicit EventBuffer(std::size_t capacity) : m_events(capacity) {
        static_assert(std::is_trivially_copyable_v<T>, "events must be trivially copyable");
    }

    bool tryPush(const T& event) noexcept {
        const std::size_t slot = m_size.fetch_add(1, std::memory_order_relaxed);
        if (slot >= m_events.size()) {
            m_dropped.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        m_events[slot] = event;
        return true;
    }

    /// Events pushed since the last clear(). Consumer thread only.
    [[nodiscard]] std::span<const T> events() const noexcept {
        return {m_events.data(), std::min(m_size.load(std::memory_order_relaxed), m_events.size())};
    }

    /// Consumer thread only.
    void clear() noexcept { m_size.store(0, std::memory_order_relaxed); }

    [[nodiscard]] std::size_t capacity() const noexcept { return m_events.size(); }
    [[nodiscard]] std::uint64_t droppedCount() const noexcept { return m_dropped.load(std::memory_order_relaxed); }

private:
    std::vector<T> m_events;
    std::atomic<std::size_t> m_size{0};
    std::atomic<std::uint64_t> m_dropped{0};
};

} // namespace frcsim
