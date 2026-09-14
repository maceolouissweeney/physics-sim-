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

constexpr float kGravity = 9.80665f;
constexpr float kControlDt = 0.004f; // 250 Hz, like a motor controller's onboard loop

World makeWorld() {
    WorldConfig config;
    config.workerThreads = 0;
    return World(config);
}

void addCarpet(World& world) {
    world.field().addGround(0.0f, world.materials().require("carpet"));
}

/// Steers every module toward `angles` with a P loop and applies `driveVolts`, for `seconds`.
/// The P gain stands in for a motor controller's onboard position loop: stiff enough that steer friction and
/// contact scrub leave well under 0.02 rad of steady-state error.
void drive(World& world, SwerveRobot& robot, const std::vector<double>& angles, float driveVolts, float seconds,
           float steerKp = 20.0f) {
    const int steps = static_cast<int>(std::lround(seconds / kControlDt));
    for (int s = 0; s < steps; ++s) {
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            const double error = std::remainder(angles[m] - robot.module(m).steerAngle, 6.283185307179586);
            const float steerVolts = std::clamp(static_cast<float>(steerKp * error), -12.0f, 12.0f);
            robot.setModuleVoltages(m, driveVolts, steerVolts);
        }
        world.step(kControlDt, 1);
    }
}

std::vector<double> straight(const SwerveRobot& robot, double angle = 0.0) {
    return std::vector<double>(robot.moduleCount(), angle);
}

SwerveDriveConfig defaultRobot() {
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    return config;
}

TEST(SwerveDrive, RestsOnCarpetAndSupportsItsWeight) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 2.0f, 2.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 2.0f);

    const RobotPose pose = robot.pose();
    EXPECT_NEAR(pose.position.GetZ(), 0.0f, 0.003f) << "robot origin should sit on the carpet";
    EXPECT_NEAR(pose.position.GetX(), 2.0f, 0.005f);
    EXPECT_NEAR(pose.position.GetY(), 2.0f, 0.005f);
    float totalNormal = 0.0f;
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        EXPECT_TRUE(robot.module(m).hasContact);
        totalNormal += robot.module(m).normalForce;
    }
    EXPECT_NEAR(totalNormal, 60.0f * kGravity, 60.0f * kGravity * 0.03f);
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
    const float wheelSpeed =
        (12.0f - m.driveFrictionTorque * motor.resistance / (m.driveEfficiency * g * motor.kt)) * motor.kv / g;
    const float expected = wheelSpeed * m.wheelRadius;
    EXPECT_NEAR(robot.pose().linearVelocity.GetX(), expected, expected * 0.03f);
    EXPECT_NEAR(robot.pose().linearVelocity.GetY(), 0.0f, 0.05f);
}

TEST(SwerveDrive, TractionLimitsAcceleration) {
    World world = makeWorld();
    addCarpet(world); // carpet friction factor 1.0
    SwerveDriveConfig config = defaultRobot();
    config.battery.internalResistance = 0.0f; // isolate traction from battery sag
    for (SwerveModuleConfig& m : config.modules) {
        m.tire = TireParams{1.0f, 1.0f, 0.1f};
        m.driveCurrentLimits = CurrentLimits{}; // stall torque far exceeds mu * N * r on every wheel
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    drive(world, robot, straight(robot), 0.0f, 0.5f); // settle
    drive(world, robot, straight(robot), 12.0f, 0.05f);
    const float v0 = robot.pose().linearVelocity.GetX();
    drive(world, robot, straight(robot), 12.0f, 0.15f);

    // Every wheel slides, so ground force is mu * N regardless of load transfer: a = mu * g.
    const float acceleration = (robot.pose().linearVelocity.GetX() - v0) / 0.15f;
    EXPECT_LT(acceleration, kGravity * 1.03f) << "cannot out-accelerate mu * g";
    EXPECT_GT(acceleration, kGravity * 0.93f);
    const float wheelSurfaceSpeed = robot.module(0).wheelVelocity * config.modules[0].wheelRadius;
    EXPECT_GT(wheelSurfaceSpeed, robot.pose().linearVelocity.GetX() + 0.1f) << "wheels should be slipping";
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
    const float v0 = robot.pose().linearVelocity.GetX();
    drive(world, robot, straight(robot), 12.0f, 0.2f);
    const float acceleration = (robot.pose().linearVelocity.GetX() - v0) / 0.2f;

    // Rear wheels gain load and are current-limited; front wheels lose load and slide at mu * N.
    //   N_front = mg/4 - m*a*h/(2L),  m*a = 2*F_limit + 2*mu*N_front
    //   => a = (2*F_limit + mu*m*g/2) / (m * (1 + mu*h/L))
    const SwerveModuleConfig& m = config.modules[0];
    const DcMotorConstants motor = deriveMotorConstants(m.driveMotor);
    const float fLimit =
        (m.driveEfficiency * m.driveGearRatio * motor.kt * m.driveCurrentLimits.stator - m.driveFrictionTorque) /
        m.wheelRadius;
    const float wheelBase = 0.55f;
    const float expected = (2.0f * fLimit + config.mass * kGravity / 2.0f) /
                           (config.mass * (1.0f + config.comHeight / wheelBase));
    EXPECT_NEAR(acceleration, expected, expected * 0.07f);

    const float front = 0.5f * (robot.module(0).normalForce + robot.module(1).normalForce);
    const float rear = 0.5f * (robot.module(2).normalForce + robot.module(3).normalForce);
    const float expectedShift = config.mass * acceleration * config.comHeight / wheelBase; // rear - front per wheel
    EXPECT_NEAR(rear - front, expectedShift, expectedShift * 0.2f);
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
    const float forcePerWheel =
        (m.driveEfficiency * m.driveGearRatio * motor.kt * 20.0f - m.driveFrictionTorque) / m.wheelRadius;
    const float expected = 4.0f * forcePerWheel / config.mass;
    const float acceleration = robot.pose().linearVelocity.GetX() / 0.3f;
    EXPECT_NEAR(acceleration, expected, expected * 0.08f);
    EXPECT_NEAR(robot.module(0).driveStatorCurrent, 20.0f, 0.5f);
}

TEST(SwerveDrive, RotatesInPlace) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 4.0f, 4.0f, 0.0f));
    std::vector<double> tangent;
    for (const SwerveModuleConfig& m : config.modules) {
        tangent.push_back(std::atan2(m.x, -m.y)); // direction of omega x r for CCW rotation
    }
    drive(world, robot, tangent, 0.0f, 0.5f);
    drive(world, robot, tangent, 6.0f, 2.0f);

    const float radius = std::hypot(config.modules[0].x, config.modules[0].y);
    const float wheelSurfaceSpeed = robot.module(0).wheelVelocity * config.modules[0].wheelRadius;
    const RobotPose pose = robot.pose();
    std::string diagnostics;
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        const SwerveModuleState& s = robot.module(m);
        const float contactRadius =
            std::hypot(s.contactX - pose.position.GetX(), s.contactY - pose.position.GetY());
        diagnostics += "\n  module " + std::to_string(m) + ": steer " + std::to_string(s.steerAngle) + " (target " +
                       std::to_string(tangent[m]) + "), wheel " + std::to_string(s.wheelVelocity) +
                       " rad/s, contact radius " + std::to_string(contactRadius) + " m, slip long " +
                       std::to_string(s.longitudinalSlip) + " lat " + std::to_string(s.lateralSlip) + " m/s";
    }
    EXPECT_NEAR(pose.angularVelocity.GetZ(), wheelSurfaceSpeed / radius, 0.05f * wheelSurfaceSpeed / radius)
        << diagnostics;
    EXPECT_NEAR(pose.position.GetX(), 4.0f, 0.05f);
    EXPECT_NEAR(pose.position.GetY(), 4.0f, 0.05f);
    EXPECT_GT(robot.continuousYaw(), 1.0) << "yaw is unwrapped and accumulates";
}

TEST(SwerveDrive, SteerTracksTargetWithinCurrentLimit) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 2.0f, 2.0f, 0.0f));
    drive(world, robot, straight(robot, 1.0), 0.0f, 0.5f);
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        EXPECT_NEAR(robot.module(m).steerAngle, 1.0, 0.02);
        EXPECT_LE(std::abs(robot.module(m).steerStatorCurrent), 40.0f + 1e-3f);
    }
    EXPECT_NEAR(robot.pose().position.GetX(), 2.0f, 0.02f) << "steering in place should not move the robot";
}

TEST(SwerveDrive, HeadOnPushingMatchIsBalanced) {
    World world = makeWorld();
    addCarpet(world);
    SwerveRobot& a = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 3.0f, 4.0f, 0.0f));
    SwerveRobot& b = world.robots().swerve(world.robots().addSwerve(defaultRobot(), 4.2f, 4.0f, JPH::JPH_PI));
    const int steps = static_cast<int>(3.0f / kControlDt);
    for (int s = 0; s < steps; ++s) {
        for (SwerveRobot* robot : {&a, &b}) {
            for (std::size_t m = 0; m < robot->moduleCount(); ++m) {
                const float steer = std::clamp(static_cast<float>(-8.0 * robot->module(m).steerAngle), -12.0f, 12.0f);
                robot->setModuleVoltages(m, 12.0f, steer);
            }
        }
        world.step(kControlDt, 1);
    }
    const RobotPose pa = a.pose();
    const RobotPose pb = b.pose();
    const float midpoint = 0.5f * (pa.position.GetX() + pb.position.GetX());
    EXPECT_NEAR(midpoint, 3.6f, 0.1f) << "identical robots should not out-push each other";
    EXPECT_LT(std::abs(pa.linearVelocity.GetX()), 0.3f);
    // Head-on pushes are unstable: robots may yaw and slide sideways, but must never interpenetrate.
    // Two 0.9 m squares cannot get their centers closer than 0.9 m.
    const float separation = std::hypot(pb.position.GetX() - pa.position.GetX(), pb.position.GetY() - pa.position.GetY());
    EXPECT_GT(separation, 0.88f) << "a: (" << pa.position.GetX() << ", " << pa.position.GetY() << ") yaw "
                                 << a.continuousYaw() << "; b: (" << pb.position.GetX() << ", " << pb.position.GetY()
                                 << ") yaw " << b.continuousYaw();
    EXPECT_LT(separation, 1.5f) << "robots should still be touching";
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
    const double startWheel = robot.module(0).wheelAngle;
    drive(world, robot, straight(robot), 12.0f, 0.5f);

    const double odometryDistance = (robot.module(0).wheelAngle - startWheel) * config.modules[0].wheelRadius;
    const double actualDistance = robot.pose().position.GetX() - 1.0;
    EXPECT_GT(actualDistance, 0.05);
    EXPECT_GT(odometryDistance, actualDistance * 1.2) << "slip must show up as odometry drift";
}

TEST(SwerveDrive, BrownoutCutsMotorPower) {
    World world = makeWorld();
    addCarpet(world);
    SwerveDriveConfig config = defaultRobot();
    config.battery.internalResistance = 0.05f;
    for (SwerveModuleConfig& m : config.modules) {
        m.driveCurrentLimits = CurrentLimits{}; // unlimited stall: 4 x ~366 A would sag far below 6.75 V
    }
    SwerveRobot& robot = world.robots().swerve(world.robots().addSwerve(config, 1.0f, 4.0f, 0.0f));
    bool sawBrownout = false;
    bool sawDisabledStep = false;
    const int steps = static_cast<int>(1.0f / kControlDt);
    for (int s = 0; s < steps; ++s) {
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            robot.setModuleVoltages(m, 12.0f, 0.0f);
        }
        const bool brownoutBeforeStep = robot.battery().brownout();
        world.step(kControlDt, 1);
        sawBrownout |= robot.battery().brownout();
        if (brownoutBeforeStep) {
            // The whole step ran browned out: outputs must have been disabled.
            ASSERT_FLOAT_EQ(robot.module(0).driveStatorCurrent, 0.0f) << "outputs disabled during brownout";
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
        std::vector<float> xyz;
        for (int i = 0; i < 60; ++i) {
            xyz.insert(xyz.end(), {6.0f + 0.2f * static_cast<float>(i % 10), 3.0f + 0.2f * static_cast<float>(i / 10),
                                   test::kFuelRadius});
        }
        world.pieces().spawn(fuel, xyz, {}, {});
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
                ASSERT_FALSE(pose.position.IsNaN() || pose.linearVelocity.IsNaN()) << "substeps " << substeps;
                ASSERT_LT(pose.position.GetZ(), 1.0f) << "robot launched, substeps " << substeps;
                for (std::size_t m = 0; m < robot->moduleCount(); ++m) {
                    ASSERT_TRUE(std::isfinite(robot->module(m).wheelVelocity));
                    ASSERT_TRUE(std::isfinite(robot->module(m).steerAngle));
                }
            }
        }
    }
}

TEST(SwerveDrive, ConfigValidation) {
    World world = makeWorld();
    SwerveDriveConfig config = defaultRobot();
    config.modules[0].x = 2.0f;
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.modules.clear();
    EXPECT_THROW(world.robots().addSwerve(config, 0, 0, 0), std::invalid_argument);
    config = defaultRobot();
    config.modules[1].tire.kineticFriction = 2.0f;
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
