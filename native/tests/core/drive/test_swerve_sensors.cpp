// Sensor imperfection models: gyro drift/scale/noise and drive encoder quantization.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "world/world.h"

namespace frcsim {
namespace {

constexpr double kTwoPi = 6.283185307179586;

// World is neither copyable nor movable: return a prvalue and add the carpet separately.
World makeWorld() {
    WorldConfig config;
    config.workerThreads = 0;
    return World(config);
}

void addCarpet(World& world) {
    world.field().addGround(0.0f, world.materials().require("carpet"));
}

void run(World& world, int periods) {
    for (int i = 0; i < periods; ++i) {
        world.step(0.020, 5);
    }
}

TEST(SwerveSensors, IdealByDefault) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(makeRectangularSwerve(0.55f, 0.55f), 2, 2, 0.7f));
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        robot.setModuleVoltages(m, 3.0f, 0.0f);
    }
    run(world, 50);
    EXPECT_DOUBLE_EQ(robot.measuredGyroYaw(), robot.continuousYaw());
    EXPECT_DOUBLE_EQ(robot.measuredDriveRotorPosition(0),
                     robot.module(0).wheelAngle * robot.config().modules[0].driveGearRatio);
}

TEST(SwerveSensors, GyroDriftAccumulatesAndResetsWithPose) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.sensors.gyroYawDriftRate = 0.01f;
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
    run(world, 500); // 10 s at rest
    EXPECT_NEAR(robot.measuredGyroYaw() - robot.continuousYaw(), 0.1, 1e-3);

    robot.resetPose(2, 2, 1.0f);
    run(world, 1);
    EXPECT_NEAR(robot.measuredGyroYaw() - robot.continuousYaw(), 0.01 * 0.020, 1e-4) << "drift restarts at reset";
}

TEST(SwerveSensors, GyroScaleErrorAppliesToRotationSinceReset) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.sensors.gyroScaleError = 0.01f;
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.5f));
    run(world, 1);
    EXPECT_NEAR(robot.measuredGyroYaw(), 0.5, 1e-4) << "no rotation yet, no scale error";

    // Rotate the chassis by +1 rad directly, then let one step observe it.
    world.physics().GetBodyInterface().SetRotation(robot.body(), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.5f),
                                                   JPH::EActivation::Activate);
    run(world, 1);
    const double rotation = robot.continuousYaw() - 0.5;
    EXPECT_NEAR(robot.measuredGyroYaw(), 0.5 + rotation * 1.01, 1e-4);
}

TEST(SwerveSensors, GyroNoiseHasConfiguredSpreadAndIsSeeded) {
    constexpr float kSigma = 0.002f;
    const auto sample = [](std::uint32_t seed) {
        World world = makeWorld();
        addCarpet(world);
        SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
        config.sensors.gyroYawNoise = kSigma;
        config.sensors.seed = seed;
        SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
        std::vector<double> errors;
        for (int i = 0; i < 1000; ++i) {
            run(world, 1);
            errors.push_back(robot.measuredGyroYaw() - robot.continuousYaw());
        }
        return errors;
    };

    const std::vector<double> a = sample(7);
    double mean = 0.0;
    double sq = 0.0;
    for (double e : a) {
        mean += e;
        sq += e * e;
    }
    mean /= static_cast<double>(a.size());
    const double stddev = std::sqrt(sq / static_cast<double>(a.size()) - mean * mean);
    EXPECT_NEAR(mean, 0.0, 3.0 * kSigma / std::sqrt(1000.0));
    EXPECT_NEAR(stddev, kSigma, kSigma * 0.1);

    EXPECT_EQ(a, sample(7)) << "same seed must reproduce the same readings";
    EXPECT_NE(a, sample(8));
}

TEST(SwerveSensors, DriveEncoderIsQuantized) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.sensors.driveEncoderCountsPerRev = 4096;
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        robot.setModuleVoltages(m, 3.0f, 0.0f);
    }
    run(world, 25);

    const double step = kTwoPi / 4096.0;
    const double measured = robot.measuredDriveRotorPosition(0);
    const double truth = robot.module(0).wheelAngle * config.modules[0].driveGearRatio;
    EXPECT_GT(truth, 10.0);
    EXPECT_NEAR(std::remainder(measured, step), 0.0, 1e-9) << "reading is a whole number of counts";
    EXPECT_GE(truth - measured, 0.0);
    EXPECT_LT(truth - measured, step);
}

} // namespace
} // namespace frcsim
