// Verifies the core performance rule: stepping a populated world does not allocate (CLAUDE.md §7).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/alloc_counter.h"
#include "core/test_support.h"
#include "field/field_json.h"
#include "world/jolt_runtime.h"
#include "world/world.h"

namespace frcsim {
namespace {

void runSteadyStateScenario(World& world) {
    loadFieldJson(world, test::readRepoFile("fields/test-flat/field.json"));
    const PieceTypeId fuel = *world.pieceTypes().find("fuel");

    // 504 fuel in a 24 x 21 grid near mid-field, plus a plow driving back and forth through them.
    std::vector<float> xyz;
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 21; ++j) {
            xyz.insert(xyz.end(), {6.0f + 0.2f * static_cast<float>(i), 2.0f + 0.2f * static_cast<float>(j),
                                   test::kFuelRadius + 0.001f});
        }
    }
    world.pieces().spawn(fuel, xyz, {}, {});
    const std::uint32_t plow = world.kinematics().addBox(JPH::Vec3(4.0f, 4.0f, 0.1f), JPH::Vec3(0.45f, 0.45f, 0.08f),
                                                         JPH::Quat::sIdentity(), world.materials().require("bumper"));

    int period = 0;
    const auto drive = [&](int periods) {
        for (int i = 0; i < periods; ++i, ++period) {
            const float t = static_cast<float>(period) * 0.020f;
            const float x = 8.3f + 4.0f * std::sin(0.5f * t); // sweeps x in [4.3, 12.3] at up to 2 m/s
            world.kinematics().moveTo(plow, JPH::Vec3(x, 4.0f, 0.1f), JPH::Quat::sIdentity(), 0.020f);
            world.step(0.020, 5);
        }
    };

    drive(150); // warm up: contact caches, broadphase, islands reach steady size

    const std::uint64_t joltBefore = joltAllocationCount();
    std::uint64_t newCalls = 0;
    {
        test::AllocationCounter counter;
        drive(150);
        newCalls = counter.count();
    }
    const std::uint64_t joltAllocations = joltAllocationCount() - joltBefore;

    EXPECT_EQ(newCalls, 0u) << "operator new called during steady-state stepping";
    // Decision D22: Jolt's broadphase rebuild (QuadTree::UpdatePrepare) makes a transient
    // `new NodeID[numBodies]` each substep. Everything else must be allocation-free.
    constexpr std::uint64_t kSubsteps = 150 * 5;
    EXPECT_LE(joltAllocations, 2 * kSubsteps) << "Jolt allocations above the known broadphase-rebuild budget";
    EXPECT_GT(world.stats().activeBodies, 1u) << "scenario should actually be pushing pieces";
}

TEST(Allocations, SteadyStateSteppingDoesNotAllocateSingleThreaded) {
    World world(WorldConfig{});
    runSteadyStateScenario(world);
}

TEST(Allocations, SteadyStateSteppingDoesNotAllocateThreadPool) {
    WorldConfig config;
    config.workerThreads = 2;
    World world(config);
    runSteadyStateScenario(world);
}

} // namespace
} // namespace frcsim
