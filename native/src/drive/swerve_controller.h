#pragma once

#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>

#include "drive/battery.h"
#include "drive/dc_motor.h"
#include "drive/swerve_config.h"
#include "world/materials.h"

namespace frcsim {

/// Live state of one module: inputs set by the user, everything else written by the controller each substep.
struct SwerveModuleState {
    // Inputs
    float driveCommandVolts = 0.0f;
    float steerCommandVolts = 0.0f;

    // Drive
    float wheelVelocityRadPerSec = 0.0f;
    double wheelAngleRadians = 0.0; ///< accumulated wheel rotation (true, without coupling or quantization)
    float driveAppliedVolts = 0.0f;
    float driveStatorCurrentAmps = 0.0f;
    float driveSupplyCurrentAmps = 0.0f;

    // Steer
    double steerAngleRadians = 0.0; ///< continuous module angle (CCW positive, 0 = robot +X)
    float steerVelocityRadPerSec = 0.0f;
    float steerAppliedVolts = 0.0f;
    float steerStatorCurrentAmps = 0.0f;
    float steerSupplyCurrentAmps = 0.0f;

    // Contact
    bool hasContact = false;
    float normalForceNewtons = 0.0f;
    float slipSpeedMetersPerSec = 0.0f;         ///< contact patch slip (magnitude)
    float longitudinalSlipMetersPerSec = 0.0f;  ///< wheel surface speed minus ground speed along the wheel
    float lateralSlipMetersPerSec = 0.0f;       ///< sideways sliding speed of the contact patch
    float contactXMeters = 0.0f;                ///< contact point, field frame
    float contactYMeters = 0.0f;
    float contactZMeters = 0.0f;
    float groundFriction = 1.0f;                ///< friction factor of the contact surface material
};

/// All mutable drivetrain state; owned by SwerveRobot, shared with the controller.
struct SwerveDrivetrainState {
    explicit SwerveDrivetrainState(const SwerveDriveConfig& config);

    std::vector<SwerveModuleState> modules;
    std::vector<DcMotorConstants> driveMotors;
    std::vector<DcMotorConstants> steerMotors;
    Battery battery;
};

/// Settings object Jolt uses to construct the controller. Not serializable.
class SwerveVehicleControllerSettings final : public JPH::VehicleControllerSettings {
public:
    SwerveVehicleControllerSettings(const SwerveDriveConfig& config, SwerveDrivetrainState& state,
                                    const MaterialTable& materials)
        : m_config(config), m_state(state), m_materials(materials) {}

    JPH::VehicleController* ConstructController(JPH::VehicleConstraint& constraint) const override;
    void SaveBinaryState(JPH::StreamOut&) const override {}
    void RestoreBinaryState(JPH::StreamIn&) override {}

private:
    const SwerveDriveConfig& m_config;
    SwerveDrivetrainState& m_state;
    const MaterialTable& m_materials;
};

/**
 * Swerve drivetrain physics inside Jolt's vehicle constraint (ADR-0003, docs/models/swerve.md):
 * steer dynamics in PreCollide, drive motors + battery in PostCollide, and the implicit tire friction
 * solve (friction circle) in SolveLongitudinalAndLateralConstraints.
 */
class SwerveVehicleController final : public JPH::VehicleController {
public:
    JPH_OVERRIDE_NEW_DELETE

    SwerveVehicleController(JPH::VehicleConstraint& constraint, const SwerveDriveConfig& config,
                            SwerveDrivetrainState& state, const MaterialTable& materials);

    JPH::Ref<JPH::VehicleControllerSettings> GetSettings() const override;

protected:
    JPH::Wheel* ConstructWheel(const JPH::WheelSettings& settings) const override;
    bool AllowSleep() const override { return false; }
    void PreCollide(float dtSeconds, JPH::PhysicsSystem& physics) override;
    void PostCollide(float dtSeconds, JPH::PhysicsSystem& physics) override;
    bool SolveLongitudinalAndLateralConstraints(float dtSeconds) override;
    void SaveState(JPH::StateRecorder&) const override {}
    void RestoreState(JPH::StateRecorder&) override {}
#ifdef JPH_DEBUG_RENDERER
    void Draw(JPH::DebugRenderer*) const override {}
#endif

private:
    [[nodiscard]] float driveInertiaKgMetersSq(std::size_t module) const;

    const SwerveDriveConfig& m_config;
    SwerveDrivetrainState& m_state;
    const MaterialTable& m_materials;
    float m_previousDtSeconds = 0.0f;
};

} // namespace frcsim
