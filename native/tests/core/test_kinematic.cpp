#include <stdexcept>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

TEST(KinematicBodies, MoveToReachesTargetAfterStep) {
    World world(WorldConfig{});
    const std::uint32_t k = world.kinematics().addBox(JPH::Vec3(0, 0, 0.1f), JPH::Vec3(0.45f, 0.45f, 0.08f),
                                                      JPH::Quat::sIdentity(), world.materials().require("bumper"));
    world.kinematics().moveTo(k, JPH::Vec3(0.06f, 0, 0.1f), JPH::Quat::sIdentity(), 0.020f);
    world.step(0.020, 5);
    EXPECT_NEAR(world.kinematics().positionMeters(k).GetX(), 0.06f, 1e-4f);
}

TEST(KinematicBodies, PlowPushesRestingPiece) {
    World world(WorldConfig{});
    world.field().addGround(0.0f, world.materials().require("carpet"));
    const PieceTypeId fuel = test::addFuelType(world);
    const std::uint32_t piece = world.pieces().spawnOne(fuel, JPH::Vec3(1.0f, 0, test::kFuelRadiusMeters));
    test::runPeriods(world, 50); // let it settle and sleep

    const std::uint32_t plow = world.kinematics().addBox(JPH::Vec3(0, 0, 0.1f), JPH::Vec3(0.45f, 0.45f, 0.08f),
                                                         JPH::Quat::sIdentity(), world.materials().require("bumper"));
    constexpr float kSpeed = 1.5f;
    float x = 0.0f;
    for (int i = 0; i < 75; ++i) { // 1.5 s
        x += kSpeed * 0.020f;
        world.kinematics().moveTo(plow, JPH::Vec3(x, 0, 0.1f), JPH::Quat::sIdentity(), 0.020f);
        world.step(0.020, 5);
    }
    EXPECT_GT(world.pieces().positionMeters(piece).GetX(), x + 0.45f)
        << "piece should be pushed ahead of the plow's front face";
}

TEST(KinematicBodies, InvalidArguments) {
    World world(WorldConfig{});
    EXPECT_THROW(world.kinematics().moveTo(0, JPH::Vec3::sZero(), JPH::Quat::sIdentity(), 0.02f), NotFoundError);
    const std::uint32_t k =
        world.kinematics().addBox(JPH::Vec3::sZero(), JPH::Vec3(1, 1, 1), JPH::Quat::sIdentity(), 0);
    EXPECT_THROW(world.kinematics().moveTo(k, JPH::Vec3::sZero(), JPH::Quat::sIdentity(), 0.0f),
                 std::invalid_argument);
}

} // namespace
} // namespace frcsim
