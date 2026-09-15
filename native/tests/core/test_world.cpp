#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include "world/layers.h"
#include "world/world.h"

namespace frcsim {
namespace {

constexpr float kFuelRadiusMeters = 0.075f; // REBUILT fuel: 0.150 m diameter
constexpr float kFuelMassKg = 0.215f;   // REBUILT fuel: 0.203-0.227 kg

JPH::BodyID addFloor(World& world) {
    JPH::BodyCreationSettings floor(new JPH::BoxShape(JPH::Vec3(10.0f, 10.0f, 0.5f)), JPH::RVec3(0, 0, -0.5),
                                    JPH::Quat::sIdentity(), JPH::EMotionType::Static, ObjectLayers::kStatic);
    return world.physics().GetBodyInterface().CreateAndAddBody(floor, JPH::EActivation::DontActivate);
}

JPH::BodyID addBall(World& world, double z) {
    JPH::BodyCreationSettings ball(new JPH::SphereShape(kFuelRadiusMeters), JPH::RVec3(0, 0, static_cast<JPH::Real>(z)),
                                   JPH::Quat::sIdentity(),
                                   JPH::EMotionType::Dynamic, ObjectLayers::kPiece);
    // Decision D7: no Jolt damping on pieces; the aero model owns air resistance.
    ball.mLinearDamping = 0.0f;
    ball.mAngularDamping = 0.0f;
    ball.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    ball.mMassPropertiesOverride.mMass = kFuelMassKg;
    return world.physics().GetBodyInterface().CreateAndAddBody(ball, JPH::EActivation::Activate);
}

void runRobotPeriods(World& world, int periods, int substeps = 5) {
    for (int i = 0; i < periods; ++i) {
        world.step(0.020, substeps);
    }
}

TEST(World, DefaultConfigCreates) {
    World world(WorldConfig{});
    EXPECT_DOUBLE_EQ(world.timeSeconds(), 0.0);
    EXPECT_EQ(world.physics().GetGravity().GetZ(), static_cast<float>(WorldConfig{}.gravityZMetersPerSecSq));
}

TEST(World, RejectsInvalidConfig) {
    WorldConfig zeroBodies;
    zeroBodies.maxBodies = 0;
    EXPECT_THROW(World{zeroBodies}, std::invalid_argument);

    WorldConfig tooManyThreads;
    tooManyThreads.workerThreads = 1000;
    EXPECT_THROW(World{tooManyThreads}, std::invalid_argument);

    WorldConfig nanGravity;
    nanGravity.gravityZMetersPerSecSq = std::nan("");
    EXPECT_THROW(World{nanGravity}, std::invalid_argument);
}

TEST(World, RejectsInvalidStepArguments) {
    World world(WorldConfig{});
    EXPECT_THROW(world.step(0.0, 5), std::invalid_argument);
    EXPECT_THROW(world.step(-0.02, 5), std::invalid_argument);
    EXPECT_THROW(world.step(std::nan(""), 5), std::invalid_argument);
    EXPECT_THROW(world.step(0.02, 0), std::invalid_argument);
    EXPECT_DOUBLE_EQ(world.timeSeconds(), 0.0);
}

TEST(World, StepAdvancesTimeAndSubsteps) {
    World world(WorldConfig{});
    runRobotPeriods(world, 50);
    EXPECT_NEAR(world.timeSeconds(), 1.0, 1e-12);
    EXPECT_EQ(world.substepCount(), 250u);
    EXPECT_EQ(world.updateErrorFlags(), 0u);
}

TEST(World, FreeFallMatchesAnalyticSolution) {
    World world(WorldConfig{});
    const double z0 = 10.0;
    const JPH::BodyID ball = addBall(world, z0);

    runRobotPeriods(world, 25); // 0.5 s at 4 ms substeps

    const double t = world.timeSeconds();
    const double expected = z0 + 0.5 * WorldConfig{}.gravityZMetersPerSecSq * t * t;
    const double actual = world.physics().GetBodyInterface().GetCenterOfMassPosition(ball).GetZ();
    // Symplectic Euler drifts by ~g*t*h/2 = 1 cm here.
    EXPECT_NEAR(actual, expected, 0.02);
}

TEST(World, BallComesToRestOnFloorAndSleeps) {
    World world(WorldConfig{});
    addFloor(world);
    const JPH::BodyID ball = addBall(world, 0.5);

    runRobotPeriods(world, 150); // 3 s

    JPH::BodyInterface& bodies = world.physics().GetBodyInterface();
    EXPECT_NEAR(bodies.GetCenterOfMassPosition(ball).GetZ(), kFuelRadiusMeters, 0.005);
    EXPECT_FALSE(bodies.IsActive(ball)) << "resting piece should be asleep";
}

TEST(World, ThreadPoolWorldSteps) {
    WorldConfig config;
    config.workerThreads = 2;
    World world(config);
    addFloor(world);
    const JPH::BodyID ball = addBall(world, 0.5);
    runRobotPeriods(world, 150);
    EXPECT_NEAR(world.physics().GetBodyInterface().GetCenterOfMassPosition(ball).GetZ(), kFuelRadiusMeters, 0.005);
}

TEST(World, MultipleWorldsAreIndependent) {
    WorldConfig lowGravity;
    lowGravity.gravityZMetersPerSecSq = -1.0;
    World a(WorldConfig{});
    World b(lowGravity);
    const JPH::BodyID ballA = addBall(a, 10.0);
    const JPH::BodyID ballB = addBall(b, 10.0);

    runRobotPeriods(a, 25);
    runRobotPeriods(b, 25);

    const double zA = a.physics().GetBodyInterface().GetCenterOfMassPosition(ballA).GetZ();
    const double zB = b.physics().GetBodyInterface().GetCenterOfMassPosition(ballB).GetZ();
    EXPECT_LT(zA, zB);
}

} // namespace
} // namespace frcsim
