// Sensor imperfection models: gyro drift/scale/noise, drive encoder quantization, and steering coupling.

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
    EXPECT_DOUBLE_EQ(robot.measuredGyroYawRadians(), robot.continuousYawRadians());
    EXPECT_DOUBLE_EQ(robot.measuredDriveRotorPositionRadians(0),
                     robot.module(0).wheelAngleRadians * robot.config().modules[0].driveGearRatio);
}

TEST(SwerveSensors, GyroDriftAccumulatesAndResetsWithPose) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.sensors.gyroYawDriftRateRadPerSec = 0.01f;
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
    run(world, 500); // 10 s at rest
    EXPECT_NEAR(robot.measuredGyroYawRadians() - robot.continuousYawRadians(), 0.1, 1e-3);

    robot.resetPose(2, 2, 1.0f);
    run(world, 1);
    EXPECT_NEAR(robot.measuredGyroYawRadians() - robot.continuousYawRadians(), 0.01 * 0.020, 1e-4)
        << "drift restarts at reset";
}

TEST(SwerveSensors, GyroScaleErrorAppliesToRotationSinceReset) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.sensors.gyroScaleError = 0.01f;
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.5f));
    run(world, 1);
    EXPECT_NEAR(robot.measuredGyroYawRadians(), 0.5, 1e-4) << "no rotation yet, no scale error";

    // Rotate the chassis by +1 rad directly, then let one step observe it.
    world.physics().GetBodyInterface().SetRotation(robot.body(), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.5f),
                                                   JPH::EActivation::Activate);
    run(world, 1);
    const double rotationRadians = robot.continuousYawRadians() - 0.5;
    EXPECT_NEAR(robot.measuredGyroYawRadians(), 0.5 + rotationRadians * 1.01, 1e-4);
}

TEST(SwerveSensors, GyroNoiseHasConfiguredSpreadAndIsSeeded) {
    constexpr float kSigmaRadians = 0.002f;
    const auto sample = [](std::uint32_t seed) {
        World world = makeWorld();
        addCarpet(world);
        SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
        config.sensors.gyroYawNoiseRadians = kSigmaRadians;
        config.sensors.seed = seed;
        SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
        std::vector<double> errorsRadians;
        for (int i = 0; i < 1000; ++i) {
            run(world, 1);
            errorsRadians.push_back(robot.measuredGyroYawRadians() - robot.continuousYawRadians());
        }
        return errorsRadians;
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
    EXPECT_NEAR(mean, 0.0, 3.0 * kSigmaRadians / std::sqrt(1000.0));
    EXPECT_NEAR(stddev, kSigmaRadians, kSigmaRadians * 0.1);

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

    const double countRadians = kTwoPi / 4096.0;
    const double measuredRadians = robot.measuredDriveRotorPositionRadians(0);
    const double truthRadians = robot.module(0).wheelAngleRadians * config.modules[0].driveGearRatio;
    EXPECT_GT(truthRadians, 10.0);
    EXPECT_NEAR(std::remainder(measuredRadians, countRadians), 0.0, 1e-9) << "reading is a whole number of counts";
    EXPECT_GE(truthRadians - measuredRadians, 0.0);
    EXPECT_LT(truthRadians - measuredRadians, countRadians);
}

TEST(SwerveSensors, SteeringCouplingMovesDriveEncoder) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    for (SwerveModuleConfig& m : config.modules) {
        m.couplingGearRatio = 50.0f / 14.0f; // MK4i
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 2, 2, 0.0f));
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        robot.setModuleVoltages(m, 0.0f, 2.0f); // steer only
    }
    run(world, 10);

    const SwerveModuleState& state = robot.module(0);
    EXPECT_GT(std::abs(state.steerAngleRadians), 0.1);
    const double expectedRadians = state.wheelAngleRadians * config.modules[0].driveGearRatio +
                                   state.steerAngleRadians * config.modules[0].couplingGearRatio;
    EXPECT_NEAR(robot.measuredDriveRotorPositionRadians(0), expectedRadians, 1e-9);
    EXPECT_GT(std::abs(robot.measuredDriveRotorPositionRadians(0)), 0.3)
        << "a steering module reports drive rotor motion even with the wheel held";
}

} // namespace
} // namespace frcsim
