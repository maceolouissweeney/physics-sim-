#include <array>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

TEST(Field, GroundTopSurfaceIsAtHeight) {
    World world(WorldConfig{});
    world.field().addGround(0.25f, world.materials().require("carpet"));
    const PieceTypeId fuel = test::addFuelType(world);
    const std::uint32_t piece = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 1.0f));
    test::runPeriods(world, 150);
    EXPECT_NEAR(world.pieces().positionMeters(piece).GetZ(), 0.25f + test::kFuelRadiusMeters, 0.005f);
}

TEST(Field, WallStopsRollingBall) {
    World world(WorldConfig{});
    world.field().addGround(0.0f, world.materials().require("carpet"));
    world.field().addBox("wall", JPH::Vec3(2.0f, 0, 0.25f), JPH::Vec3(0.05f, 2.0f, 0.25f), JPH::Quat::sIdentity(),
                         world.materials().require("polycarbonate"));
    const PieceTypeId fuel = test::addFuelType(world);
    const std::uint32_t piece =
        world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, test::kFuelRadiusMeters), JPH::Vec3(3.0f, 0, 0));

    float maxX = 0.0f;
    for (int i = 0; i < 100; ++i) {
        test::runPeriods(world, 1);
        maxX = std::max(maxX, world.pieces().positionMeters(piece).GetX());
    }
    EXPECT_LT(maxX, 1.95f - test::kFuelRadiusMeters + 0.02f) << "ball passed through the wall";
    EXPECT_GT(maxX, 1.5f) << "ball never reached the wall";
}

TEST(Field, CylinderAxisIsVertical) {
    World world(WorldConfig{});
    // Vertical post: radius 0.3, 1 m tall, top at z = 1.
    world.field().addCylinder("post", JPH::Vec3(0, 0, 0.5f), 0.3f, 0.5f, JPH::Quat::sIdentity(),
                              world.materials().require("aluminum"));
    const PieceTypeId fuel = test::addFuelType(world);
    const std::uint32_t piece = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 2.0f));
    test::runPeriods(world, 100);
    EXPECT_NEAR(world.pieces().positionMeters(piece).GetZ(), 1.0f + test::kFuelRadiusMeters, 0.01f)
        << "ball should rest on the top face of a Z-axis cylinder";
}

TEST(Field, ConvexHullIsSolid) {
    World world(WorldConfig{});
    const std::array<JPH::Vec3, 8> cube = {
        JPH::Vec3(-0.5f, -0.5f, 0), JPH::Vec3(0.5f, -0.5f, 0), JPH::Vec3(0.5f, 0.5f, 0), JPH::Vec3(-0.5f, 0.5f, 0),
        JPH::Vec3(-0.5f, -0.5f, 1), JPH::Vec3(0.5f, -0.5f, 1), JPH::Vec3(0.5f, 0.5f, 1), JPH::Vec3(-0.5f, 0.5f, 1),
    };
    world.field().addConvexHull("block", JPH::Vec3::sZero(), cube, JPH::Quat::sIdentity(),
                                world.materials().require("aluminum"));
    const PieceTypeId fuel = test::addFuelType(world);
    const std::uint32_t piece = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 2.0f));
    test::runPeriods(world, 100);
    EXPECT_NEAR(world.pieces().positionMeters(piece).GetZ(), 1.0f + test::kFuelRadiusMeters, 0.01f);
}

TEST(Field, RejectsInvalidGeometry) {
    World world(WorldConfig{});
    const MaterialId m = MaterialTable::kDefault;
    EXPECT_THROW(world.field().addBox("b", JPH::Vec3::sZero(), JPH::Vec3(0, 1, 1), JPH::Quat::sIdentity(), m),
                 std::invalid_argument);
    EXPECT_THROW(world.field().addCylinder("c", JPH::Vec3::sZero(), -1.0f, 1.0f, JPH::Quat::sIdentity(), m),
                 std::invalid_argument);
    const std::array<JPH::Vec3, 3> tooFew = {JPH::Vec3::sZero(), JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY()};
    EXPECT_THROW(world.field().addConvexHull("h", JPH::Vec3::sZero(), tooFew, JPH::Quat::sIdentity(), m),
                 std::invalid_argument);
    EXPECT_THROW(world.field().addBox("b", JPH::Vec3::sZero(), JPH::Vec3(1, 1, 1), JPH::Quat(0, 0, 0, 2), m),
                 std::invalid_argument);
    EXPECT_THROW(world.field().addBox("b", JPH::Vec3::sZero(), JPH::Vec3(1, 1, 1), JPH::Quat::sIdentity(), 42),
                 NotFoundError);
    EXPECT_EQ(world.field().size(), 0u);
}

TEST(Field, ClearRemovesBodies) {
    World world(WorldConfig{});
    world.field().addGround(0.0f, MaterialTable::kDefault);
    world.field().addBox("b", JPH::Vec3(0, 0, 1), JPH::Vec3(1, 1, 1), JPH::Quat::sIdentity(), 0);
    EXPECT_EQ(world.physics().GetNumBodies(), 2u);
    world.field().clear();
    EXPECT_EQ(world.physics().GetNumBodies(), 0u);
    EXPECT_EQ(world.field().size(), 0u);
}

TEST(Field, BoundsValidation) {
    World world(WorldConfig{});
    EXPECT_THROW(world.field().setBoundsMeters(JPH::AABox(JPH::Vec3(1, 1, 1), JPH::Vec3(0, 0, 0))), std::invalid_argument);
    world.field().setBoundsMeters(JPH::AABox(JPH::Vec3(0, 0, 0), JPH::Vec3(1, 1, 1)));
    EXPECT_FLOAT_EQ(world.field().boundsMeters().mMax.GetX(), 1.0f);
}

} // namespace
} // namespace frcsim
