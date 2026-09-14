#pragma once

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include "drive/swerve_config.h"
#include "drive/swerve_controller.h"
#include "world/materials.h"

namespace frcsim {

struct RobotPose {
    JPH::Vec3 position;        ///< robot frame origin (on the carpet under the frame center)
    JPH::Quat rotation;
    JPH::Vec3 linearVelocity;  ///< of the center of mass, field frame
    JPH::Vec3 angularVelocity; ///< field frame
};

/// A swerve robot: chassis body + Jolt vehicle constraint + SwerveVehicleController (ADR-0003).
class SwerveRobot {
public:
    SwerveRobot(JPH::PhysicsSystem& physics, const MaterialTable& materials, const SwerveDriveConfig& config,
                std::uint32_t index, float x, float y, float yaw);
    ~SwerveRobot();

    SwerveRobot(const SwerveRobot&) = delete;
    SwerveRobot& operator=(const SwerveRobot&) = delete;

    /// Commands held until changed. Throws std::invalid_argument / NotFoundError.
    void setModuleVoltages(std::size_t module, float driveVolts, float steerVolts);

    [[nodiscard]] std::size_t moduleCount() const { return m_config->modules.size(); }
    [[nodiscard]] const SwerveModuleState& module(std::size_t index) const;
    [[nodiscard]] const SwerveDriveConfig& config() const { return *m_config; }
    [[nodiscard]] const Battery& battery() const { return m_state->battery; }
    [[nodiscard]] JPH::BodyID body() const { return m_body; }

    [[nodiscard]] RobotPose pose() const;
    /// True unwrapped yaw (rad) since creation or the last resetPose(); updated by postStep().
    [[nodiscard]] double continuousYaw() const { return m_continuousYaw; }

    /// Gyro reading: true yaw with scale error, drift, and noise (SensorParams). Equals continuousYaw() when ideal.
    [[nodiscard]] double measuredGyroYaw() const { return m_gyroYaw; }

    /// Drive encoder reading (rotor rad), quantized to SensorParams::driveEncoderCountsPerRev.
    [[nodiscard]] double measuredDriveRotorPosition(std::size_t module) const;

    /// Places the robot on the carpet at (x, y) facing yaw, at rest. Also re-zeros the gyro (reads yaw, drift restarts).
    void resetPose(float x, float y, float yaw);

    /// Once per World::step after all substeps; dt is the step duration.
    void postStep(double dt);

private:
    JPH::PhysicsSystem& m_physics;
    std::unique_ptr<SwerveDriveConfig> m_config;    // stable addresses: the controller references these
    std::unique_ptr<SwerveDrivetrainState> m_state;
    JPH::BodyID m_body;
    JPH::Ref<JPH::VehicleCollisionTester> m_tester;
    JPH::Ref<JPH::VehicleConstraint> m_constraint;
    double m_continuousYaw = 0.0;
    float m_lastYaw = 0.0f;

    // Gyro model
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normal{0.0, 1.0};
    double m_gyroOrigin = 0.0;    ///< true yaw at the last gyro reset
    double m_gyroElapsed = 0.0;   ///< seconds since the last gyro reset (drift)
    double m_gyroYaw = 0.0;
};

/// All robots in a world.
class Robots {
public:
    Robots(JPH::PhysicsSystem& physics, const MaterialTable& materials);

    std::uint32_t addSwerve(const SwerveDriveConfig& config, float x, float y, float yaw);
    [[nodiscard]] SwerveRobot& swerve(std::uint32_t index);
    [[nodiscard]] const SwerveRobot& swerve(std::uint32_t index) const;
    [[nodiscard]] std::size_t size() const { return m_swerve.size(); }

    void postStep(double dt);

private:
    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<std::unique_ptr<SwerveRobot>> m_swerve;
};

} // namespace frcsim
