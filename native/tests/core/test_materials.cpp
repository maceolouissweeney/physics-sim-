#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "util/errors.h"
#include "world/materials.h"
#include "world/world.h"

namespace frcsim {
namespace {

TEST(MaterialTable, DefaultMaterialExists) {
    MaterialTable table;
    EXPECT_EQ(table.size(), 1u);
    EXPECT_EQ(table.require("default"), MaterialTable::kDefault);
}

TEST(MaterialTable, AddFindAndUpdate) {
    MaterialTable table;
    const MaterialId rubber = table.add("rubber", Material{1.1f, 0.6f});
    EXPECT_EQ(table.find("rubber"), rubber);
    EXPECT_FLOAT_EQ(table.get(rubber).friction, 1.1f);

    EXPECT_EQ(table.add("rubber", Material{0.9f, 0.4f}), rubber) << "re-adding updates in place";
    EXPECT_FLOAT_EQ(table.get(rubber).friction, 0.9f);
    EXPECT_FALSE(table.find("missing"));
    EXPECT_THROW((void)table.require("missing"), NotFoundError);
}

TEST(MaterialTable, DefaultCombinationRules) {
    MaterialTable table;
    const MaterialId a = table.add("a", Material{0.25f, 0.2f});
    const MaterialId b = table.add("b", Material{1.0f, 0.7f});
    const Material ab = table.combined(a, b);
    EXPECT_FLOAT_EQ(ab.friction, 0.5f);    // sqrt(0.25 * 1.0)
    EXPECT_FLOAT_EQ(ab.restitution, 0.7f); // max
    EXPECT_FLOAT_EQ(table.combined(b, a).friction, ab.friction);
}

TEST(MaterialTable, PairOverrideIsSymmetricAndSurvivesUpdates) {
    MaterialTable table;
    const MaterialId a = table.add("a", Material{0.25f, 0.2f});
    const MaterialId b = table.add("b", Material{1.0f, 0.7f});
    table.setPair(a, b, Material{0.1f, 0.05f});
    EXPECT_FLOAT_EQ(table.combined(a, b).friction, 0.1f);
    EXPECT_FLOAT_EQ(table.combined(b, a).restitution, 0.05f);

    table.add("a", Material{0.5f, 0.5f}); // updating a material must not clobber the override
    EXPECT_FLOAT_EQ(table.combined(a, b).friction, 0.1f);
    EXPECT_FLOAT_EQ(table.combined(a, a).friction, 0.5f);
}

TEST(MaterialTable, RejectsInvalidValuesAndOverflow) {
    MaterialTable table;
    EXPECT_THROW(table.add("bad", Material{-0.1f, 0.0f}), std::invalid_argument);
    EXPECT_THROW(table.add("bad", Material{0.5f, 1.5f}), std::invalid_argument);
    EXPECT_THROW(table.add("", Material{}), std::invalid_argument);
    for (std::size_t i = table.size(); i < MaterialTable::kMaxMaterials; ++i) {
        table.add("m" + std::to_string(i), Material{});
    }
    EXPECT_THROW(table.add("one-too-many", Material{}), CapacityExceededError);
}

/// Drops a ball onto the ground and returns the height of the first bounce apex (ball bottom above ground).
float firstBounceApex(World& world, PieceTypeId fuel, float dropHeight) {
    const std::uint32_t piece = world.pieces().spawnOne(fuel, JPH::Vec3(0, 0, dropHeight + test::kFuelRadius));
    const JPH::BodyID body = world.pieces().body(piece);
    JPH::BodyInterface& bodies = world.physics().GetBodyInterface();

    bool rising = false;
    float apex = 0.0f;
    for (int i = 0; i < 3000; ++i) { // up to 6 s at 2 ms
        world.step(0.002, 1);
        const float vz = bodies.GetLinearVelocity(body).GetZ();
        const float bottom = static_cast<float>(bodies.GetPosition(body).GetZ()) - test::kFuelRadius;
        if (!rising && vz > 0.05f) {
            rising = true;
        }
        if (rising) {
            apex = std::max(apex, bottom);
            if (vz < 0.0f) {
                break;
            }
        }
    }
    return apex;
}

TEST(MaterialContacts, RestitutionControlsBounceHeight) {
    World world(WorldConfig{});
    const MaterialId ground = world.materials().add("test-ground", Material{0.5f, 0.6f});
    world.field().addGround(0.0f, ground);
    const PieceTypeId fuel = test::addFuelType(world);
    world.materials().add("foam", Material{0.5f, 0.6f});

    // Combined restitution 0.6: apex ~= e^2 * h.
    const float apex = firstBounceApex(world, fuel, 1.0f);
    EXPECT_NEAR(apex, 0.36f, 0.06f);
}

TEST(MaterialContacts, PairOverrideRemovesBounce) {
    World world(WorldConfig{});
    const MaterialId ground = world.materials().add("test-ground", Material{0.5f, 0.6f});
    world.field().addGround(0.0f, ground);
    const PieceTypeId fuel = test::addFuelType(world);
    const MaterialId foam = world.materials().add("foam", Material{0.5f, 0.6f});
    world.materials().setPair(foam, ground, Material{0.5f, 0.0f});

    EXPECT_LT(firstBounceApex(world, fuel, 1.0f), 0.01f);
}

} // namespace
} // namespace frcsim
