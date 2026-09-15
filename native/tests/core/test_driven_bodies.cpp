#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

DrivenBodies::BoxDesc robotDesc(float x, float y) {
    DrivenBodies::BoxDesc desc;
    desc.centerMeters = JPH::Vec3(x, y, 0.11f);
    desc.massKg = 60.0f;
    desc.maxForceNewtons = 300.0f; // 5 m/s^2 max acceleration
    return desc;
}

TEST(DrivenBodies, AccelerationIsForceLimited) {
    World world(WorldConfig{});
    const std::uint32_t robot = world.driven().addBox(robotDesc(0, 0));
    world.driven().setTargetVelocity(robot, 3.0f, 0.0f, 0.0f);

    world.step(0.2, 50); // 0.2 s at 5 m/s^2 -> 1.0 m/s
    EXPECT_NEAR(world.driven().velocityMetersPerSec(robot).GetX(), 1.0f, 0.02f);

    world.step(1.0, 250); // reaches and holds the 3 m/s target
    EXPECT_NEAR(world.driven().velocityMetersPerSec(robot).GetX(), 3.0f, 0.01f);
    EXPECT_NEAR(world.driven().positionMeters(robot).GetZ(), 0.11f, 1e-4f) << "planar motion only";
}

TEST(DrivenBodies, StallsAgainstWallInsteadOfTunneling) {
    World world(WorldConfig{});
    world.field().addBox("wall", JPH::Vec3(1.0f, 0, 0.5f), JPH::Vec3(0.05f, 2.0f, 0.5f), JPH::Quat::sIdentity(),
                         world.materials().require("polycarbonate"));
    const std::uint32_t robot = world.driven().addBox(robotDesc(0, 0));
    world.driven().setTargetVelocity(robot, 4.0f, 0.0f, 0.0f);
    test::runPeriods(world, 150); // 3 s of pushing into the wall

    EXPECT_LT(world.driven().positionMeters(robot).GetX(), 0.95f - 0.45f + 0.02f);
    EXPECT_NEAR(world.driven().velocityMetersPerSec(robot).GetX(), 0.0f, 0.05f);
}

TEST(DrivenBodies, PushesPiecesWithoutLaunchingThem) {
    World world(WorldConfig{});
    world.field().addGround(0.0f, world.materials().require("carpet"));
    const PieceTypeId fuel = test::addFuelType(world);
    std::vector<float> xyz;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            xyz.insert(xyz.end(), {1.0f + 0.16f * static_cast<float>(i), -0.8f + 0.16f * static_cast<float>(j),
                                   test::kFuelRadiusMeters + 0.001f});
        }
    }
    world.pieces().spawn(fuel, xyz, {}, {});
    DrivenBodies::BoxDesc desc = robotDesc(0, 0);
    desc.maxForceNewtons = 590.0f;
    const std::uint32_t robot = world.driven().addBox(desc);
    world.driven().setTargetVelocity(robot, 2.0f, 0.0f, 0.0f);

    float maxZ = 0.0f;
    for (int period = 0; period < 150; ++period) {
        world.step(0.020, 5);
        for (std::uint32_t i = 0; i < world.pieces().highWater(); ++i) {
            maxZ = std::max(maxZ, world.pieces().positionMeters(i).GetZ());
        }
    }
    EXPECT_GT(world.driven().positionMeters(robot).GetX(), 1.0f) << "robot should drive into the pile";
    EXPECT_LT(maxZ, 1.0f) << "pieces must not be launched by the pusher";
}

TEST(DrivenBodies, InvalidArguments) {
    World world(WorldConfig{});
    DrivenBodies::BoxDesc desc = robotDesc(0, 0);
    desc.massKg = 0.0f;
    EXPECT_THROW(world.driven().addBox(desc), std::invalid_argument);
    EXPECT_THROW(world.driven().setTargetVelocity(3, 1, 0, 0), NotFoundError);
    const std::uint32_t robot = world.driven().addBox(robotDesc(0, 0));
    EXPECT_THROW(world.driven().setTargetVelocity(robot, NAN, 0, 0), std::invalid_argument);
}

} // namespace
} // namespace frcsim
