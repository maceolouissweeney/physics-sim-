#include <algorithm>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "util/event_buffer.h"

namespace frcsim {
namespace {

TEST(EventBuffer, PushReadClear) {
    EventBuffer<int> buffer(4);
    EXPECT_TRUE(buffer.tryPush(1));
    EXPECT_TRUE(buffer.tryPush(2));
    ASSERT_EQ(buffer.events().size(), 2u);
    EXPECT_EQ(buffer.events()[1], 2);
    buffer.clear();
    EXPECT_TRUE(buffer.events().empty());
}

TEST(EventBuffer, OverflowDropsAndCounts) {
    EventBuffer<int> buffer(2);
    EXPECT_TRUE(buffer.tryPush(1));
    EXPECT_TRUE(buffer.tryPush(2));
    EXPECT_FALSE(buffer.tryPush(3));
    EXPECT_FALSE(buffer.tryPush(4));
    EXPECT_EQ(buffer.events().size(), 2u);
    EXPECT_EQ(buffer.droppedCount(), 2u);
    buffer.clear();
    EXPECT_TRUE(buffer.tryPush(5));
}

TEST(EventBuffer, ConcurrentProducersLoseNothingWithinCapacity) {
    constexpr int kThreads = 4;
    constexpr int kPerThread = 5000;
    EventBuffer<int> buffer(kThreads * kPerThread);

    std::vector<std::thread> producers;
    for (int t = 0; t < kThreads; ++t) {
        producers.emplace_back([&buffer, t] {
            for (int i = 0; i < kPerThread; ++i) {
                buffer.tryPush(t * kPerThread + i);
            }
        });
    }
    for (auto& p : producers) {
        p.join();
    }

    std::vector<int> values(buffer.events().begin(), buffer.events().end());
    ASSERT_EQ(values.size(), static_cast<std::size_t>(kThreads * kPerThread));
    std::sort(values.begin(), values.end());
    for (int i = 0; i < kThreads * kPerThread; ++i) {
        ASSERT_EQ(values[static_cast<std::size_t>(i)], i);
    }
    EXPECT_EQ(buffer.droppedCount(), 0u);
}

} // namespace
} // namespace frcsim
