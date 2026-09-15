#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

class PiecePoolTest : public ::testing::Test {
protected:
    PiecePoolTest() : world(makeConfig()) {
        world.field().addGround(0.0f, world.materials().require("carpet"));
        fuel = test::addFuelType(world);
    }

    static WorldConfig makeConfig() {
        WorldConfig config;
        config.maxPieces = 8;
        config.maxBodies = 64;
        return config;
    }

    World world;
    PieceTypeId fuel = 0;
};

TEST_F(PiecePoolTest, BatchSpawnAssignsIndicesAndOutputs) {
    const std::vector<float> xyz = {0, 0, 1, 1, 0, 1, 2, 0, 1};
    std::vector<std::uint32_t> indices(3);
    world.pieces().spawn(fuel, xyz, {}, indices);

    EXPECT_EQ(indices, (std::vector<std::uint32_t>{0, 1, 2}));
    EXPECT_EQ(world.pieces().highWater(), 3u);
    EXPECT_EQ(world.pieces().countInState(PieceState::OnField), 3u);
    EXPECT_FLOAT_EQ(world.pieces().positionsMetersData()[3], 1.0f); // piece 1 x, before any step
    EXPECT_EQ(world.pieces().statesData()[2], static_cast<std::uint8_t>(PieceState::OnField));

    test::runPeriods(world, 100);
    EXPECT_NEAR(world.pieces().positionsMetersData()[5], test::kFuelRadiusMeters, 0.005f) << "outputs refresh after step";
}

TEST_F(PiecePoolTest, SpawnIsAllOrNothingAtCapacity) {
    std::vector<float> xyz;
    for (int i = 0; i < 6; ++i) {
        xyz.insert(xyz.end(), {static_cast<float>(i), 0, 1});
    }
    world.pieces().spawn(fuel, xyz, {}, {});
    EXPECT_THROW(world.pieces().spawn(fuel, xyz, {}, {}), CapacityExceededError);
    EXPECT_EQ(world.pieces().highWater(), 6u);
    EXPECT_EQ(world.pieces().countInState(PieceState::OnField), 6u);
}

TEST_F(PiecePoolTest, DespawnRecyclesBodyForSameType) {
    const std::uint32_t a = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 1));
    const JPH::BodyID body = world.pieces().body(a);
    world.pieces().despawn(a);
    EXPECT_EQ(world.pieces().state(a), PieceState::Inactive);
    EXPECT_FALSE(world.physics().GetBodyInterface().IsAdded(body));
    EXPECT_EQ(world.physics().GetNumBodies(), 2u) << "body is kept for reuse (ground + piece)";

    const std::uint32_t b = world.pieces().spawnOne(fuel, JPH::Vec3(3, 0, 1));
    EXPECT_EQ(b, a);
    EXPECT_EQ(world.pieces().body(b), body);
    EXPECT_TRUE(world.physics().GetBodyInterface().IsAdded(body));
    EXPECT_NEAR(world.pieces().positionMeters(b).GetX(), 3.0f, 1e-5f);
}

TEST_F(PiecePoolTest, DifferentTypeDoesNotReuseBody) {
    const std::uint32_t a = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 1));
    world.pieces().despawn(a);

    PieceTypeDesc box;
    box.name = "box";
    box.shape = PieceShape::Box;
    box.halfExtentsMeters = {0.1f, 0.1f, 0.1f};
    box.massKg = 1.0f;
    const PieceTypeId boxType = world.pieceTypes().add(box, world.materials());
    const std::uint32_t b = world.pieces().spawnOne(boxType, JPH::Vec3(0, 0, 1));
    EXPECT_NE(b, a);
    EXPECT_EQ(world.pieces().type(b), boxType);
}

TEST_F(PiecePoolTest, NonPhysicalStatesRemoveBody) {
    const std::uint32_t p = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, 1));
    const JPH::BodyID body = world.pieces().body(p);
    JPH::BodyInterface& bodies = world.physics().GetBodyInterface();

    world.pieces().setState(p, PieceState::InRobot);
    EXPECT_FALSE(bodies.IsAdded(body));
    EXPECT_EQ(world.pieces().countInState(PieceState::InRobot), 1u);
    test::runPeriods(world, 10); // must not touch removed bodies

    world.pieces().teleport(p, JPH::Vec3(1, 1, 0.5f), JPH::Vec3(0, 0, 0), JPH::Vec3::sZero());
    EXPECT_EQ(world.pieces().state(p), PieceState::OnField);
    EXPECT_TRUE(bodies.IsAdded(body));

    EXPECT_THROW(world.pieces().setState(p, PieceState::Inactive), std::invalid_argument);
}

TEST_F(PiecePoolTest, OutOfBoundsPiecesAreRemoved) {
    world.field().setBoundsMeters(JPH::AABox(JPH::Vec3(-1, -1, -1), JPH::Vec3(1, 1, 5)));
    const std::uint32_t p =
        world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, test::kFuelRadiusMeters), JPH::Vec3(5.0f, 0, 0));
    test::runPeriods(world, 50);
    EXPECT_EQ(world.pieces().state(p), PieceState::OutOfBounds);
    EXPECT_FALSE(world.physics().GetBodyInterface().IsAdded(world.pieces().body(p)));
    EXPECT_EQ(world.stats().piecesSimulated, 0u);
}

TEST_F(PiecePoolTest, InvalidArguments) {
    EXPECT_THROW(world.pieces().despawn(0), NotFoundError);
    EXPECT_THROW(world.pieces().spawn(99, std::vector<float>{0, 0, 1}, {}, {}), NotFoundError);
    EXPECT_THROW(world.pieces().spawn(fuel, std::vector<float>{0, 0}, {}, {}), std::invalid_argument);
    EXPECT_THROW(world.pieces().spawn(fuel, std::vector<float>{0, 0, NAN}, {}, {}), std::invalid_argument);
    EXPECT_THROW(world.pieces().spawn(fuel, std::vector<float>{0, 0, 1}, std::vector<float>{1}, {}),
                 std::invalid_argument);
}

TEST(PiecePoolBodies, MaxBodiesExhaustionLeavesStateUnchanged) {
    WorldConfig config;
    config.maxBodies = 12;
    config.maxPieces = 12;
    World world(config);
    const PieceTypeId fuel = test::addFuelType(world);
    for (int i = 0; i < 6; ++i) {
        world.field().addBox("b", JPH::Vec3(static_cast<float>(i) * 3, 5, 0), JPH::Vec3(1, 1, 1),
                             JPH::Quat::sIdentity(), 0);
    }
    const std::vector<float> xyz(3 * 7, 1.0f);
    EXPECT_THROW(world.pieces().spawn(fuel, xyz, {}, {}), CapacityExceededError);
    EXPECT_EQ(world.pieces().highWater(), 0u);
    EXPECT_EQ(world.physics().GetNumBodies(), 6u);
}

TEST(PieceTypes, Validation) {
    World world(WorldConfig{});
    PieceTypeDesc desc;
    desc.name = "ball";
    desc.radiusMeters = 0.1f;
    desc.massKg = 0.0f;
    EXPECT_THROW(world.pieceTypes().add(desc, world.materials()), std::invalid_argument);
    desc.massKg = 1.0f;
    desc.material = 63;
    EXPECT_THROW(world.pieceTypes().add(desc, world.materials()), NotFoundError);
    desc.material = 0;
    world.pieceTypes().add(desc, world.materials());
    EXPECT_THROW(world.pieceTypes().add(desc, world.materials()), std::invalid_argument) << "duplicate name";
}

TEST(PieceTypes, CylinderPieceRestsOnFlatFace) {
    World world(WorldConfig{});
    world.field().addGround(0.0f, MaterialTable::kDefault);
    PieceTypeDesc desc;
    desc.name = "puck";
    desc.shape = PieceShape::Cylinder;
    desc.radiusMeters = 0.15f;
    desc.halfHeightMeters = 0.025f;
    desc.massKg = 0.3f;
    const PieceTypeId puck = world.pieceTypes().add(desc, world.materials());
    const std::uint32_t p = world.pieces().spawnOne(puck, JPH::Vec3(0, 0, 0.3f));
    test::runPeriods(world, 100);
    EXPECT_NEAR(world.pieces().positionMeters(p).GetZ(), 0.025f, 0.005f);
}

} // namespace
} // namespace frcsim
