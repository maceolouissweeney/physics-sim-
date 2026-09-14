#include "drive/swerve_controller.h"

#include <algorithm>
#include <cmath>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/Wheel.h>

#include "drive/tire_model.h"
#include "world/body_tag.h"

namespace frcsim {
namespace {

constexpr double kTwoPi = 6.283185307179586;

/// Wraps to [-pi, pi], the range Jolt expects for Wheel::SetSteerAngle.
float wrapAngle(double angle) {
    return static_cast<float>(std::remainder(angle, kTwoPi));
}

} // namespace

SwerveDrivetrainState::SwerveDrivetrainState(const SwerveDriveConfig& config)
    : modules(config.modules.size()), battery(config.battery) {
    driveMotors.reserve(config.modules.size());
    steerMotors.reserve(config.modules.size());
    for (const SwerveModuleConfig& m : config.modules) {
        driveMotors.push_back(deriveMotorConstants(m.driveMotor));
        steerMotors.push_back(deriveMotorConstants(m.steerMotor));
    }
}

JPH::VehicleController* SwerveVehicleControllerSettings::ConstructController(JPH::VehicleConstraint& constraint) const {
    return new SwerveVehicleController(constraint, m_config, m_state, m_materials);
}

SwerveVehicleController::SwerveVehicleController(JPH::VehicleConstraint& constraint, const SwerveDriveConfig& config,
                                                 SwerveDrivetrainState& state, const MaterialTable& materials)
    : JPH::VehicleController(constraint), m_config(config), m_state(state), m_materials(materials) {}

JPH::Ref<JPH::VehicleControllerSettings> SwerveVehicleController::GetSettings() const {
    return new SwerveVehicleControllerSettings(m_config, m_state, m_materials);
}

JPH::Wheel* SwerveVehicleController::ConstructWheel(const JPH::WheelSettings& settings) const {
    return new JPH::Wheel(settings);
}

float SwerveVehicleController::driveInertia(std::size_t i) const {
    const SwerveModuleConfig& cfg = m_config.modules[i];
    return cfg.wheelInertia + cfg.driveGearRatio * cfg.driveGearRatio * m_state.driveMotors[i].rotorInertia;
}

void SwerveVehicleController::PreCollide(float dt, JPH::PhysicsSystem&) {
    const bool brownout = m_state.battery.brownout();
    const float busVoltage = m_state.battery.voltage();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const SwerveModuleConfig& cfg = m_config.modules[i];
        SwerveModuleState& st = m_state.modules[i];
        JPH::Wheel* wheel = wheels[i];

        // Encoders integrate the post-solve wheel speed of the previous step.
        st.wheelVelocity = wheel->GetAngularVelocity();
        st.wheelAngle += static_cast<double>(st.wheelVelocity) * m_previousDt;

        // Normal force from last step's suspension impulse.
        st.hasContact = wheel->HasContact();
        st.normalForce = st.hasContact && m_previousDt > 0.0f
                             ? std::max(0.0f, wheel->GetSuspensionLambda() / m_previousDt)
                             : 0.0f;

        // Steer axis: motor + gearbox + module inertia; scrub resists turning a loaded wheel.
        const DcMotorConstants& motor = m_state.steerMotors[i];
        const float inertia = cfg.steerInertia + cfg.steerGearRatio * cfg.steerGearRatio * motor.rotorInertia;
        const float scrub = cfg.tire.staticFriction * st.normalForce * cfg.scrubRadius * st.groundFriction;
        const MotorStepResult r = stepGearedMotor(
            motor, GearboxParams{cfg.steerGearRatio, cfg.steerEfficiency, cfg.steerFrictionTorque + scrub}, inertia,
            st.steerVelocity, brownout ? 0.0f : st.steerVoltageCommand, busVoltage,
            brownout ? NeutralMode::Coast : cfg.steerNeutralMode, cfg.steerCurrentLimits, 0.0f, dt);
        st.steerVelocity = r.velocity;
        st.steerAngle += static_cast<double>(r.velocity) * dt;
        st.steerAppliedVoltage = r.appliedVoltage;
        st.steerStatorCurrent = r.statorCurrent;
        st.steerSupplyCurrent = r.supplyCurrent;

        // Contact axes for this step are computed from the steer angle right after PreCollide.
        wheel->SetSteerAngle(wrapAngle(st.steerAngle));
    }
}

void SwerveVehicleController::PostCollide(float dt, JPH::PhysicsSystem& physics) {
    const bool brownout = m_state.battery.brownout();
    const float busVoltage = m_state.battery.voltage();
    const float impulseScale = m_previousDt > 0.0f ? dt / m_previousDt : 0.0f;
    const JPH::BodyLockInterfaceNoLock& locks = physics.GetBodyLockInterfaceNoLock();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    float totalSupplyCurrent = 0.0f;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const SwerveModuleConfig& cfg = m_config.modules[i];
        SwerveModuleState& st = m_state.modules[i];
        JPH::Wheel* wheel = wheels[i];

        // Ground material scales tire friction (carpet = 1.0).
        st.groundFriction = 1.0f;
        if (wheel->HasContact()) {
            const JPH::RVec3 contact = wheel->GetContactPosition();
            st.contactX = static_cast<float>(contact.GetX());
            st.contactY = static_cast<float>(contact.GetY());
            st.contactZ = static_cast<float>(contact.GetZ());
            if (const JPH::Body* ground = locks.TryGetBody(wheel->GetContactBodyID())) {
                const MaterialId material = BodyTag::decodeMaterial(ground->GetUserData());
                st.groundFriction = m_materials.combined(material, material).friction;
            }
        }

        // Drive motor, stepped implicitly against last step's ground impulse (as Jolt's wheeled controller does).
        const float radius = cfg.wheelRadius;
        const float inertia = driveInertia(i);
        const float groundAngularImpulse =
            wheel->HasContact() ? -impulseScale * wheel->GetLongitudinalLambda() * radius : 0.0f;
        const MotorStepResult d = stepGearedMotor(
            m_state.driveMotors[i], GearboxParams{cfg.driveGearRatio, cfg.driveEfficiency, cfg.driveFrictionTorque},
            inertia, wheel->GetAngularVelocity(), brownout ? 0.0f : st.driveVoltageCommand, busVoltage,
            brownout ? NeutralMode::Coast : cfg.driveNeutralMode, cfg.driveCurrentLimits,
            dt > 0.0f ? groundAngularImpulse / dt : 0.0f, dt);
        // Remove the load estimate again: the solver applies the actual ground impulse.
        wheel->SetAngularVelocity(d.velocity - groundAngularImpulse / inertia);

        st.driveAppliedVoltage = d.appliedVoltage;
        st.driveStatorCurrent = d.statorCurrent;
        st.driveSupplyCurrent = d.supplyCurrent;
        totalSupplyCurrent += d.supplyCurrent + st.steerSupplyCurrent;
    }

    m_state.battery.update(totalSupplyCurrent);
    m_previousDt = dt;
}

bool SwerveVehicleController::SolveLongitudinalAndLateralConstraints(float) {
    bool appliedImpulse = false;
    const JPH::Body* chassis = mConstraint.GetVehicleBody();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    for (std::size_t i = 0; i < wheels.size(); ++i) {
        JPH::Wheel* wheel = wheels[i];
        SwerveModuleState& st = m_state.modules[i];
        if (!wheel->HasContact()) {
            st.slipSpeed = 0.0f;
            st.longitudinalSlip = 0.0f;
            st.lateralSlip = 0.0f;
            continue;
        }
        const SwerveModuleConfig& cfg = m_config.modules[i];
        const float radius = cfg.wheelRadius;
        const float inertia = driveInertia(i);

        const JPH::Vec3 relativeVelocity =
            chassis->GetPointVelocity(wheel->GetContactPosition()) - wheel->GetContactPointVelocity();
        const float vLong = relativeVelocity.Dot(wheel->GetContactLongitudinal());
        const float vLat = relativeVelocity.Dot(wheel->GetContactLateral());
        const float omega = wheel->GetAngularVelocity();
        const float longitudinalSlip = omega * radius - vLong;
        st.longitudinalSlip = longitudinalSlip;
        st.lateralSlip = vLat;
        st.slipSpeed = std::sqrt(longitudinalSlip * longitudinalSlip + vLat * vLat);

        const float mu = tireFriction(cfg.tire, st.slipSpeed) * st.groundFriction;
        const float maxImpulse = mu * std::max(0.0f, wheel->GetSuspensionLambda());

        // Longitudinal: the impulse that zeroes contact slip through the wheel inertia, clamped by friction.
        const float previous = wheel->GetLongitudinalLambda();
        const float target =
            std::clamp(previous + (omega - vLong / radius) * inertia / radius, -maxImpulse, maxImpulse);
        appliedImpulse |= wheel->SolveLongitudinalConstraintPart(mConstraint, target, target);
        wheel->SetAngularVelocity(omega - (wheel->GetLongitudinalLambda() - previous) * radius / inertia);

        // Lateral: whatever the friction circle has left.
        const float lateralMax = std::sqrt(std::max(0.0f, maxImpulse * maxImpulse - target * target));
        appliedImpulse |= wheel->SolveLateralConstraintPart(mConstraint, -lateralMax, lateralMax);
    }
    return appliedImpulse;
}

} // namespace frcsim
