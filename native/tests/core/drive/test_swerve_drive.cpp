// Validation suite for the swerve model (IMPLEMENTATION_PLAN P2.10, docs/models/swerve.md).

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <Jolt/Jolt.h>

#include "core/test_support.h"
#include "field/field_json.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

constexpr float kGravityMetersPerSecSq = 9.80665f;
constexpr float kControlDtSeconds = 0.004f; // 250 Hz, like a motor controller's onboard loop

World makeWorld() {
    WorldConfig config;
    config.workerThreads = 0;
    return World(config);
}

void addCarpet(World& world) {
    world.field().addGround(0.0f, world.materials().require("carpet"));
}

/// Steers every module toward `anglesRadians` with a P loop and applies `driveVolts`, for `seconds`.
/// The P gain stands in for a motor controller's onboard position loop: stiff enough that steer friction and
/// contact scrub leave well under 0.02 rad of steady-state error.
void drive(World& world, SwerveRobot& robot, const std::vector<double>& anglesRadians, float driveVolts,
           float seconds, float steerKpVoltsPerRadian = 20.0f) {
    const int steps = static_cast<int>(std::lround(seconds / kControlDtSeconds));
    for (int s = 0; s < steps; ++s) {
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            const double errorRadians =
                std::remainder(anglesRadians[m] - robot.module(m).steerAngleRadians, 6.283185307179586);
            const float steerVolts = std::clamp(static_cast<float>(steerKpVoltsPerRadian * errorRadians), -12.0f, 12.0f);
            robot.setModuleVoltages(m, driveVolts, steerVolts);
        }
        world.step(kControlDtSeconds, 1);
    }
}

std::vector<double> straight(const SwerveRobot& robot, double angleRadians = 0.0) {
    return std::vector<double>(robot.moduleCount(), angleRadians);
}

SwerveDriveConfig defaultRobot() {
    return makeRectangularSwerve(0.55f, 0.55f);
}

TEST(SwerveDrive, RestsOnCarpetAndSupportsItsWeight) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 2.0f, 2.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 2.0f);

    const RobotPose pose = robot.pose();
    EXPECT_NEAR(pose.positionMeters.GetZ(), 0.0f, 0.003f) << "robot origin should sit on the carpet";
    EXPECT_NEAR(pose.positionMeters.GetX(), 2.0f, 0.005f);
    EXPECT_NEAR(pose.positionMeters.GetY(), 2.0f, 0.005f);
    float totalNormalNewtons = 0.0f;
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        EXPECT_TRUE(robot.module(m).hasContact);
        totalNormalNewtons += robot.module(m).normalForceNewtons;
    }
    EXPECT_NEAR(totalNormalNewtons, 60.0f * kGravityMetersPerSecSq, 60.0f * kGravityMetersPerSecSq * 0.03f);
}

TEST(SwerveDrive, FreeSpeedMatchesMotorModel) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 12.0f, 4.0f);

    // Steady state: eta*G*Kt*(V - G*w/Kv)/R = friction  =>  w = (V - friction*R/(eta*G*Kt)) * Kv / G
    const SwerveModuleConfig& m = config.modules[0];
    const DcMotorConstants motor = deriveMotorConstants(m.driveMotor);
    const float g = m.driveGearRatio;
    const float wheelSpeedRadPerSec = (12.0f - m.driveFrictionTorqueNewtonMeters * motor.resistanceOhms /
                                                   (m.driveEfficiency * g * motor.ktNewtonMetersPerAmp)) *
                                      motor.kvRadPerSecPerVolt / g;
    const float expectedMetersPerSec = wheelSpeedRadPerSec * m.wheelRadiusMeters;
    EXPECT_NEAR(robot.pose().linearVelocityMetersPerSec.GetX(), expectedMetersPerSec, expectedMetersPerSec * 0.03f);
    EXPECT_NEAR(robot.pose().linearVelocityMetersPerSec.GetY(), 0.0f, 0.05f);
}

TEST(SwerveDrive, TractionLimitsAcceleration) {
    World world = makeWorld();
    addCarpet(world); // carpet friction factor 1.0
    SwerveDriveConfig config = defaultRobot();
    config.battery.internalResistanceOhms = 0.0f; // isolate traction from battery sag
    for (SwerveModuleConfig& m : config.modules) {
        m.tire = TireParams{1.0f, 1.0f, 0.1f};
        m.driveCurrentLimits = CurrentLimits{}; // stall torque far exceeds mu * N * r on every wheel
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 0.5f); // settle
    drive(world, robot, straight(robot), 12.0f, 0.05f);
    const float v0MetersPerSec = robot.pose().linearVelocityMetersPerSec.GetX();
    drive(world, robot, straight(robot), 12.0f, 0.15f);

    // Every wheel slides, so ground force is mu * N regardless of load transfer: a = mu * g.
    const float accelerationMetersPerSecSq = (robot.pose().linearVelocityMetersPerSec.GetX() - v0MetersPerSec) / 0.15f;
    EXPECT_LT(accelerationMetersPerSecSq, kGravityMetersPerSecSq * 1.03f) << "cannot out-accelerate mu * g";
    EXPECT_GT(accelerationMetersPerSecSq, kGravityMetersPerSecSq * 0.93f);
    const float wheelSurfaceSpeedMetersPerSec =
        robot.module(0).wheelVelocityRadPerSec * config.modules[0].wheelRadiusMeters;
    EXPECT_GT(wheelSurfaceSpeedMetersPerSec, robot.pose().linearVelocityMetersPerSec.GetX() + 0.1f)
        << "wheels should be slipping";
}

TEST(SwerveDrive, LoadTransferUnloadsFrontWheels) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot(); // 80 A stator limit
    for (SwerveModuleConfig& m : config.modules) {
        m.tire = TireParams{1.0f, 1.0f, 0.1f};
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 0.5f);
    drive(world, robot, straight(robot), 12.0f, 0.05f);
    const float v0MetersPerSec = robot.pose().linearVelocityMetersPerSec.GetX();
    drive(world, robot, straight(robot), 12.0f, 0.2f);
    const float accelerationMetersPerSecSq = (robot.pose().linearVelocityMetersPerSec.GetX() - v0MetersPerSec) / 0.2f;

    // Rear wheels gain load and are current-limited; front wheels lose load and slide at mu * N.
    //   N_front = mg/4 - m*a*h/(2L),  m*a = 2*F_limit + 2*mu*N_front
    //   => a = (2*F_limit + mu*m*g/2) / (m * (1 + mu*h/L))
    const SwerveModuleConfig& m = config.modules[0];
    const DcMotorConstants motor = deriveMotorConstants(m.driveMotor);
    const float limitForceNewtons = (m.driveEfficiency * m.driveGearRatio * motor.ktNewtonMetersPerAmp *
                                         m.driveCurrentLimits.statorAmps -
                                     m.driveFrictionTorqueNewtonMeters) /
                                    m.wheelRadiusMeters;
    const float wheelBaseMeters = 0.55f;
    const float expectedMetersPerSecSq =
        (2.0f * limitForceNewtons + config.massKg * kGravityMetersPerSecSq / 2.0f) /
        (config.massKg * (1.0f + config.comHeightMeters / wheelBaseMeters));
    EXPECT_NEAR(accelerationMetersPerSecSq, expectedMetersPerSecSq, expectedMetersPerSecSq * 0.07f);

    const float frontNewtons = 0.5f * (robot.module(0).normalForceNewtons + robot.module(1).normalForceNewtons);
    const float rearNewtons = 0.5f * (robot.module(2).normalForceNewtons + robot.module(3).normalForceNewtons);
    const float expectedShiftNewtons = // rear - front per wheel
        config.massKg * accelerationMetersPerSecSq * config.comHeightMeters / wheelBaseMeters;
    EXPECT_NEAR(rearNewtons - frontNewtons, expectedShiftNewtons, expectedShiftNewtons * 0.2f);
}

TEST(SwerveDrive, CurrentLimitSetsAcceleration) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    for (SwerveModuleConfig& m : config.modules) {
        m.driveCurrentLimits = CurrentLimits{20.0f, 0.0f};
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 0.5f);
    drive(world, robot, straight(robot), 12.0f, 0.3f);

    const SwerveModuleConfig& m = config.modules[0];
    const DcMotorConstants motor = deriveMotorConstants(m.driveMotor);
    const float forcePerWheelNewtons =
        (m.driveEfficiency * m.driveGearRatio * motor.ktNewtonMetersPerAmp * 20.0f - m.driveFrictionTorqueNewtonMeters) /
        m.wheelRadiusMeters;
    const float expectedMetersPerSecSq = 4.0f * forcePerWheelNewtons / config.massKg;
    const float accelerationMetersPerSecSq = robot.pose().linearVelocityMetersPerSec.GetX() / 0.3f;
    EXPECT_NEAR(accelerationMetersPerSecSq, expectedMetersPerSecSq, expectedMetersPerSecSq * 0.08f);
    EXPECT_NEAR(robot.module(0).driveStatorCurrentAmps, 20.0f, 0.5f);
}

TEST(SwerveDrive, RotatesInPlace) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 4.0f, 4.0f, 0.0f));
    std::vector<double> tangentRadians;
    for (const SwerveModuleConfig& m : config.modules) {
        tangentRadians.push_back(std::atan2(m.xMeters, -m.yMeters)); // direction of omega x r for CCW rotation
    }
    drive(world, robot, tangentRadians, 0.0f, 0.5f);
    drive(world, robot, tangentRadians, 6.0f, 2.0f);

    const float radiusMeters = std::hypot(config.modules[0].xMeters, config.modules[0].yMeters);
    const float wheelSurfaceSpeedMetersPerSec =
        robot.module(0).wheelVelocityRadPerSec * config.modules[0].wheelRadiusMeters;
    const RobotPose pose = robot.pose();
    std::string diagnostics;
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        const SwerveModuleState& s = robot.module(m);
        const float contactRadiusMeters = std::hypot(s.contactXMeters - pose.positionMeters.GetX(),
                                                     s.contactYMeters - pose.positionMeters.GetY());
        diagnostics += "\n  module " + std::to_string(m) + ": steer " + std::to_string(s.steerAngleRadians) +
                       " (target " + std::to_string(tangentRadians[m]) + "), wheel " +
                       std::to_string(s.wheelVelocityRadPerSec) + " rad/s, contact radius " +
                       std::to_string(contactRadiusMeters) + " m, slip long " +
                       std::to_string(s.longitudinalSlipMetersPerSec) + " lat " +
                       std::to_string(s.lateralSlipMetersPerSec) + " m/s";
    }
    const float expectedYawRateRadPerSec = wheelSurfaceSpeedMetersPerSec / radiusMeters;
    EXPECT_NEAR(pose.angularVelocityRadPerSec.GetZ(), expectedYawRateRadPerSec, 0.05f * expectedYawRateRadPerSec)
        << diagnostics;
    EXPECT_NEAR(pose.positionMeters.GetX(), 4.0f, 0.05f);
    EXPECT_NEAR(pose.positionMeters.GetY(), 4.0f, 0.05f);
    EXPECT_GT(robot.continuousYawRadians(), 1.0) << "yaw is unwrapped and accumulates";
}

TEST(SwerveDrive, SteerTracksTargetWithinCurrentLimit) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 2.0f, 2.0f, 0.0f));
    drive(world, robot, straight(robot, 1.0), 0.0f, 0.5f);
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        EXPECT_NEAR(robot.module(m).steerAngleRadians, 1.0, 0.02);
        EXPECT_LE(std::abs(robot.module(m).steerStatorCurrentAmps), 40.0f + 1e-3f);
    }
    EXPECT_NEAR(robot.pose().positionMeters.GetX(), 2.0f, 0.02f) << "steering in place should not move the robot";
}

TEST(SwerveDrive, HeadOnPushingMatchIsBalanced) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& a = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 3.0f, 4.0f, 0.0f));
    SwerveRobot& b = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 4.2f, 4.0f, JPH::JPH_PI));
    const int steps = static_cast<int>(3.0f / kControlDtSeconds);
    for (int s = 0; s < steps; ++s) {
        for (SwerveRobot* robot : {&a, &b}) {
            for (std::size_t m = 0; m < robot->moduleCount(); ++m) {
                const float steerVolts =
                    std::clamp(static_cast<float>(-8.0 * robot->module(m).steerAngleRadians), -12.0f, 12.0f);
                robot->setModuleVoltages(m, 12.0f, steerVolts);
            }
        }
        world.step(kControlDtSeconds, 1);
    }
    const RobotPose pa = a.pose();
    const RobotPose pb = b.pose();
    const float midpointMeters = 0.5f * (pa.positionMeters.GetX() + pb.positionMeters.GetX());
    EXPECT_NEAR(midpointMeters, 3.6f, 0.1f) << "identical robots should not out-push each other";
    EXPECT_LT(std::abs(pa.linearVelocityMetersPerSec.GetX()), 0.3f);
    // Head-on pushes are unstable: robots may yaw and slide sideways, but must never interpenetrate.
    // Two 0.9 m squares cannot get their centers closer than 0.9 m.
    const float separationMeters = std::hypot(pb.positionMeters.GetX() - pa.positionMeters.GetX(),
                                              pb.positionMeters.GetY() - pa.positionMeters.GetY());
    EXPECT_GT(separationMeters, 0.88f) << "a: (" << pa.positionMeters.GetX() << ", " << pa.positionMeters.GetY()
                                       << ") yaw " << a.continuousYawRadians() << "; b: (" << pb.positionMeters.GetX()
                                       << ", " << pb.positionMeters.GetY() << ") yaw " << b.continuousYawRadians();
    EXPECT_LT(separationMeters, 1.5f) << "robots should still be touching";
}

TEST(SwerveDrive, WheelSlipMakesOdometryOverestimateDistance) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    for (SwerveModuleConfig& m : config.modules) {
        m.tire = TireParams{0.3f, 0.25f, 0.1f};
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 0.3f);
    const double startWheelRadians = robot.module(0).wheelAngleRadians;
    drive(world, robot, straight(robot), 12.0f, 0.5f);

    const double odometryDistanceMeters =
        (robot.module(0).wheelAngleRadians - startWheelRadians) * config.modules[0].wheelRadiusMeters;
    const double actualDistanceMeters = robot.pose().positionMeters.GetX() - 1.0;
    EXPECT_GT(actualDistanceMeters, 0.05);
    EXPECT_GT(odometryDistanceMeters, actualDistanceMeters * 1.2) << "slip must show up as odometry drift";
}

TEST(SwerveDrive, BrownoutCutsMotorPower) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    config.battery.internalResistanceOhms = 0.05f;
    for (SwerveModuleConfig& m : config.modules) {
        m.driveCurrentLimits = CurrentLimits{}; // unlimited stall: 4 x ~366 A would sag far below 6.75 V
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    bool sawBrownout = false;
    bool sawDisabledStep = false;
    const int steps = static_cast<int>(1.0f / kControlDtSeconds);
    for (int s = 0; s < steps; ++s) {
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            robot.setModuleVoltages(m, 12.0f, 0.0f);
        }
        const bool brownoutBeforeStep = robot.battery().brownout();
        world.step(kControlDtSeconds, 1);
        sawBrownout |= robot.battery().brownout();
        if (brownoutBeforeStep) {
            // The whole step ran browned out: outputs must have been disabled.
            ASSERT_FLOAT_EQ(robot.module(0).driveStatorCurrentAmps, 0.0f) << "outputs disabled during brownout";
            sawDisabledStep = true;
        }
    }
    EXPECT_TRUE(sawBrownout);
    EXPECT_TRUE(sawDisabledStep);
}

TEST(SwerveDrive, RandomizedDrivingStaysFinite) {
    for (int substeps : {1, 5, 10}) {
        World world = makeWorld();
        loadFieldJson(world, test::readRepoFile("fields/test-flat/field.json"));
        const PieceTypeId fuel = *world.pieceTypes().find("fuel");
        std::vector<float> positionsXyzMeters;
        for (int i = 0; i < 60; ++i) {
            positionsXyzMeters.insert(positionsXyzMeters.end(),
                                      {6.0f + 0.2f * static_cast<float>(i % 10),
                                       3.0f + 0.2f * static_cast<float>(i / 10), test::kFuelRadiusMeters});
        }
        world.pieces().spawn(fuel, positionsXyzMeters, {}, {});
        SwerveRobot& a = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 4.0f, 4.0f, 0.0f));
        SwerveRobot& b = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 9.0f, 4.0f, 1.0f));

        std::mt19937 rng(1234u + static_cast<unsigned>(substeps));
        std::uniform_real_distribution<float> volts(-12.0f, 12.0f);
        for (int period = 0; period < 750; ++period) { // 15 s
            if (period % 25 == 0) {
                for (SwerveRobot* robot : {&a, &b}) {
                    for (std::size_t m = 0; m < robot->moduleCount(); ++m) {
                        robot->setModuleVoltages(m, volts(rng), volts(rng));
                    }
                }
            }
            world.step(0.020, substeps);
            for (SwerveRobot* robot : {&a, &b}) {
                const RobotPose pose = robot->pose();
                ASSERT_FALSE(pose.positionMeters.IsNaN() || pose.linearVelocityMetersPerSec.IsNaN())
                    << "substeps " << substeps;
                ASSERT_LT(pose.positionMeters.GetZ(), 1.0f) << "robot launched, substeps " << substeps;
                for (std::size_t m = 0; m < robot->moduleCount(); ++m) {
                    ASSERT_TRUE(std::isfinite(robot->module(m).wheelVelocityRadPerSec));
                    ASSERT_TRUE(std::isfinite(robot->module(m).steerAngleRadians));
                }
            }
        }
    }
}

TEST(SwerveDrive, ConfigValidation) {
    World world = makeWorld();
    SwerveDriveConfig config = defaultRobot();
    config.modules[0].xMeters = 2.0f;
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.modules.clear();
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.modules[1].tire.kineticFriction = 2.0f;
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.modules[2].couplingGearRatio = NAN;
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.bumperMaterial = 60;
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), NotFoundError);
    EXPECT_THROW((void)world.robots().swerve(0), NotFoundError);
    const std::uint32_t index = world.robots().addSwerve(defaultRobot(), 0, 0, 0);
    EXPECT_THROW(world.robots().swerve(index).setModuleVoltages(4, 1, 1), NotFoundError);
    EXPECT_THROW(world.robots().swerve(index).setModuleVoltages(0, NAN, 1), std::invalid_argument);
}

} // namespace
} // namespace frcsim
