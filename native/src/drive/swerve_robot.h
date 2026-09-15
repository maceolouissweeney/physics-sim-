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
    JPH::Vec3 positionMeters;               ///< robot frame origin (on the carpet under the frame center)
    JPH::Quat rotation;
    JPH::Vec3 linearVelocityMetersPerSec;   ///< of the center of mass, field frame
    JPH::Vec3 angularVelocityRadPerSec;     ///< field frame
};

/// A swerve robot: chassis body + Jolt vehicle constraint + SwerveVehicleController (ADR-0003).
class SwerveRobot {
public:
    SwerveRobot(JPH::PhysicsSystem& physics, const MaterialTable& materials, const SwerveDriveConfig& config,
                std::uint32_t index, float xMeters, float yMeters, float yawRadians);
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
    /// True unwrapped yaw since creation or the last resetPose(); updated by postStep().
    [[nodiscard]] double continuousYawRadians() const { return m_continuousYawRadians; }

    /// Gyro reading: true yaw with scale error, drift, and noise (SensorParams). Equals the true yaw when ideal.
    [[nodiscard]] double measuredGyroYawRadians() const { return m_gyroYawRadians; }

    /// Drive encoder reading: wheel rotation * gear ratio + module rotation * coupling ratio, quantized to
    /// SensorParams::driveEncoderCountsPerRev.
    [[nodiscard]] double measuredDriveRotorPositionRadians(std::size_t module) const;

    /// Drive rotor velocity including the steering coupling term.
    [[nodiscard]] double driveRotorVelocityRadPerSec(std::size_t module) const;

    /// Places the robot on the carpet at rest and re-zeros the gyro to yawRadians (drift restarts).
    void resetPose(float xMeters, float yMeters, float yawRadians);

    /// Once per World::step after all substeps.
    void postStep(double dtSeconds);

private:
    JPH::PhysicsSystem& m_physics;
    std::unique_ptr<SwerveDriveConfig> m_config;    // stable addresses: the controller references these
    std::unique_ptr<SwerveDrivetrainState> m_state;
    JPH::BodyID m_body;
    JPH::Ref<JPH::VehicleCollisionTester> m_tester;
    JPH::Ref<JPH::VehicleConstraint> m_constraint;
    double m_continuousYawRadians = 0.0;
    float m_lastYawRadians = 0.0f;

    // Gyro model
    std::mt19937 m_rng;
    std::normal_distribution<double> m_standardNormal{0.0, 1.0};
    double m_gyroOriginRadians = 0.0;  ///< true yaw at the last gyro reset
    double m_gyroElapsedSeconds = 0.0; ///< time since the last gyro reset (drift)
    double m_gyroYawRadians = 0.0;
};

/// All robots in a world.
class Robots {
public:
    Robots(JPH::PhysicsSystem& physics, const MaterialTable& materials);

    std::uint32_t addSwerve(const SwerveDriveConfig& config, float xMeters, float yMeters, float yawRadians);
    [[nodiscard]] SwerveRobot& swerve(std::uint32_t index);
    [[nodiscard]] const SwerveRobot& swerve(std::uint32_t index) const;
    [[nodiscard]] std::size_t size() const { return m_swerve.size(); }

    void postStep(double dtSeconds);

private:
    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<std::unique_ptr<SwerveRobot>> m_swerve;
};

} // namespace frcsim
