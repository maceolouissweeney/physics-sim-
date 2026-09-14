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
    float driveVoltageCommand = 0.0f;
    float steerVoltageCommand = 0.0f;

    // Drive
    float wheelVelocity = 0.0f; ///< rad/s
    double wheelAngle = 0.0;    ///< rad, accumulated (drive encoder = wheelAngle * gear ratio)
    float driveAppliedVoltage = 0.0f;
    float driveStatorCurrent = 0.0f;
    float driveSupplyCurrent = 0.0f;

    // Steer
    double steerAngle = 0.0;    ///< rad, continuous module angle (CCW positive, 0 = robot +X)
    float steerVelocity = 0.0f; ///< rad/s at the module
    float steerAppliedVoltage = 0.0f;
    float steerStatorCurrent = 0.0f;
    float steerSupplyCurrent = 0.0f;

    // Contact
    bool hasContact = false;
    float normalForce = 0.0f;       ///< N
    float slipSpeed = 0.0f;         ///< m/s at the contact patch (magnitude)
    float longitudinalSlip = 0.0f;  ///< m/s, wheel surface speed minus ground speed along the wheel (+ = spinning)
    float lateralSlip = 0.0f;       ///< m/s, sideways sliding speed of the contact patch
    float contactX = 0.0f;          ///< contact point, field frame (m)
    float contactY = 0.0f;
    float contactZ = 0.0f;
    float groundFriction = 1.0f;    ///< friction factor of the contact surface material
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
    void PreCollide(float dt, JPH::PhysicsSystem& physics) override;
    void PostCollide(float dt, JPH::PhysicsSystem& physics) override;
    bool SolveLongitudinalAndLateralConstraints(float dt) override;
    void SaveState(JPH::StateRecorder&) const override {}
    void RestoreState(JPH::StateRecorder&) override {}
#ifdef JPH_DEBUG_RENDERER
    void Draw(JPH::DebugRenderer*) const override {}
#endif

private:
    [[nodiscard]] float driveInertia(std::size_t module) const;

    const SwerveDriveConfig& m_config;
    SwerveDrivetrainState& m_state;
    const MaterialTable& m_materials;
    float m_previousDt = 0.0f;
};

} // namespace frcsim
