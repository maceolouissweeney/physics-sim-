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
float wrapAngleRadians(double angleRadians) {
    return static_cast<float>(std::remainder(angleRadians, kTwoPi));
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

float SwerveVehicleController::driveInertiaKgMetersSq(std::size_t i) const {
    const SwerveModuleConfig& cfg = m_config.modules[i];
    return cfg.wheelInertiaKgMetersSq +
           cfg.driveGearRatio * cfg.driveGearRatio * m_state.driveMotors[i].rotorInertiaKgMetersSq;
}

void SwerveVehicleController::PreCollide(float dtSeconds, JPH::PhysicsSystem&) {
    const bool brownout = m_state.battery.brownout();
    const float busVolts = m_state.battery.voltageVolts();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const SwerveModuleConfig& cfg = m_config.modules[i];
        SwerveModuleState& st = m_state.modules[i];
        JPH::Wheel* wheel = wheels[i];

        // Encoders integrate the post-solve wheel speed of the previous step.
        st.wheelVelocityRadPerSec = wheel->GetAngularVelocity();
        st.wheelAngleRadians += static_cast<double>(st.wheelVelocityRadPerSec) * m_previousDtSeconds;

        // Normal force from last step's suspension impulse.
        st.hasContact = wheel->HasContact();
        st.normalForceNewtons = st.hasContact && m_previousDtSeconds > 0.0f
                                    ? std::max(0.0f, wheel->GetSuspensionLambda() / m_previousDtSeconds)
                                    : 0.0f;

        // Steer axis: motor + gearbox + module inertia; scrub resists turning a loaded wheel.
        const DcMotorConstants& motor = m_state.steerMotors[i];
        const float inertiaKgMetersSq =
            cfg.steerInertiaKgMetersSq + cfg.steerGearRatio * cfg.steerGearRatio * motor.rotorInertiaKgMetersSq;
        const float scrubTorqueNewtonMeters =
            cfg.tire.staticFriction * st.normalForceNewtons * cfg.scrubRadiusMeters * st.groundFriction;
        const MotorStepResult r = stepGearedMotor(
            motor,
            GearboxParams{cfg.steerGearRatio, cfg.steerEfficiency,
                          cfg.steerFrictionTorqueNewtonMeters + scrubTorqueNewtonMeters},
            inertiaKgMetersSq, st.steerVelocityRadPerSec, brownout ? 0.0f : st.steerCommandVolts, busVolts,
            brownout ? NeutralMode::Coast : cfg.steerNeutralMode, cfg.steerCurrentLimits, 0.0f, dtSeconds);
        st.steerVelocityRadPerSec = r.velocityRadPerSec;
        st.steerAngleRadians += static_cast<double>(r.velocityRadPerSec) * dtSeconds;
        st.steerAppliedVolts = r.appliedVolts;
        st.steerStatorCurrentAmps = r.statorCurrentAmps;
        st.steerSupplyCurrentAmps = r.supplyCurrentAmps;

        // Contact axes for this step are computed from the steer angle right after PreCollide.
        wheel->SetSteerAngle(wrapAngleRadians(st.steerAngleRadians));
    }
}

void SwerveVehicleController::PostCollide(float dtSeconds, JPH::PhysicsSystem& physics) {
    const bool brownout = m_state.battery.brownout();
    const float busVolts = m_state.battery.voltageVolts();
    const float impulseScale = m_previousDtSeconds > 0.0f ? dtSeconds / m_previousDtSeconds : 0.0f;
    const JPH::BodyLockInterfaceNoLock& locks = physics.GetBodyLockInterfaceNoLock();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    float totalSupplyCurrentAmps = 0.0f;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const SwerveModuleConfig& cfg = m_config.modules[i];
        SwerveModuleState& st = m_state.modules[i];
        JPH::Wheel* wheel = wheels[i];

        // Ground material scales tire friction (carpet = 1.0).
        st.groundFriction = 1.0f;
        if (wheel->HasContact()) {
            const JPH::RVec3 contact = wheel->GetContactPosition();
            st.contactXMeters = static_cast<float>(contact.GetX());
            st.contactYMeters = static_cast<float>(contact.GetY());
            st.contactZMeters = static_cast<float>(contact.GetZ());
            if (const JPH::Body* ground = locks.TryGetBody(wheel->GetContactBodyID())) {
                const MaterialId material = BodyTag::decodeMaterial(ground->GetUserData());
                st.groundFriction = m_materials.combined(material, material).friction;
            }
        }

        // Drive motor, stepped implicitly against last step's ground impulse (as Jolt's wheeled controller does).
        const float radiusMeters = cfg.wheelRadiusMeters;
        const float inertiaKgMetersSq = driveInertiaKgMetersSq(i);
        const float groundAngularImpulseNewtonMeterSec =
            wheel->HasContact() ? -impulseScale * wheel->GetLongitudinalLambda() * radiusMeters : 0.0f;
        const MotorStepResult d = stepGearedMotor(
            m_state.driveMotors[i],
            GearboxParams{cfg.driveGearRatio, cfg.driveEfficiency, cfg.driveFrictionTorqueNewtonMeters},
            inertiaKgMetersSq, wheel->GetAngularVelocity(), brownout ? 0.0f : st.driveCommandVolts, busVolts,
            brownout ? NeutralMode::Coast : cfg.driveNeutralMode, cfg.driveCurrentLimits,
            dtSeconds > 0.0f ? groundAngularImpulseNewtonMeterSec / dtSeconds : 0.0f, dtSeconds);
        // Remove the load estimate again: the solver applies the actual ground impulse.
        wheel->SetAngularVelocity(d.velocityRadPerSec - groundAngularImpulseNewtonMeterSec / inertiaKgMetersSq);

        st.driveAppliedVolts = d.appliedVolts;
        st.driveStatorCurrentAmps = d.statorCurrentAmps;
        st.driveSupplyCurrentAmps = d.supplyCurrentAmps;
        totalSupplyCurrentAmps += d.supplyCurrentAmps + st.steerSupplyCurrentAmps;
    }

    m_state.battery.update(totalSupplyCurrentAmps);
    m_previousDtSeconds = dtSeconds;
}

bool SwerveVehicleController::SolveLongitudinalAndLateralConstraints(float) {
    bool appliedImpulse = false;
    const JPH::Body* chassis = mConstraint.GetVehicleBody();
    JPH::Wheels& wheels = mConstraint.GetWheels();

    for (std::size_t i = 0; i < wheels.size(); ++i) {
        JPH::Wheel* wheel = wheels[i];
        SwerveModuleState& st = m_state.modules[i];
        if (!wheel->HasContact()) {
            st.slipSpeedMetersPerSec = 0.0f;
            st.longitudinalSlipMetersPerSec = 0.0f;
            st.lateralSlipMetersPerSec = 0.0f;
            continue;
        }
        const SwerveModuleConfig& cfg = m_config.modules[i];
        const float radiusMeters = cfg.wheelRadiusMeters;
        const float inertiaKgMetersSq = driveInertiaKgMetersSq(i);

        const JPH::Vec3 relativeVelocityMetersPerSec =
            chassis->GetPointVelocity(wheel->GetContactPosition()) - wheel->GetContactPointVelocity();
        const float longitudinalVelocityMetersPerSec = relativeVelocityMetersPerSec.Dot(wheel->GetContactLongitudinal());
        const float lateralVelocityMetersPerSec = relativeVelocityMetersPerSec.Dot(wheel->GetContactLateral());
        const float omegaRadPerSec = wheel->GetAngularVelocity();
        st.longitudinalSlipMetersPerSec = omegaRadPerSec * radiusMeters - longitudinalVelocityMetersPerSec;
        st.lateralSlipMetersPerSec = lateralVelocityMetersPerSec;
        st.slipSpeedMetersPerSec = std::sqrt(st.longitudinalSlipMetersPerSec * st.longitudinalSlipMetersPerSec +
                                             lateralVelocityMetersPerSec * lateralVelocityMetersPerSec);

        const float frictionCoefficient = tireFriction(cfg.tire, st.slipSpeedMetersPerSec) * st.groundFriction;
        const float maxImpulseNewtonSec = frictionCoefficient * std::max(0.0f, wheel->GetSuspensionLambda());

        // Longitudinal: the impulse that zeroes contact slip through the wheel inertia, clamped by friction.
        const float previousImpulseNewtonSec = wheel->GetLongitudinalLambda();
        const float targetImpulseNewtonSec = std::clamp(
            previousImpulseNewtonSec +
                (omegaRadPerSec - longitudinalVelocityMetersPerSec / radiusMeters) * inertiaKgMetersSq / radiusMeters,
            -maxImpulseNewtonSec, maxImpulseNewtonSec);
        appliedImpulse |= wheel->SolveLongitudinalConstraintPart(mConstraint, targetImpulseNewtonSec, targetImpulseNewtonSec);
        wheel->SetAngularVelocity(omegaRadPerSec - (wheel->GetLongitudinalLambda() - previousImpulseNewtonSec) *
                                                       radiusMeters / inertiaKgMetersSq);

        // Lateral: whatever the friction circle has left.
        const float lateralMaxImpulseNewtonSec = std::sqrt(std::max(
            0.0f, maxImpulseNewtonSec * maxImpulseNewtonSec - targetImpulseNewtonSec * targetImpulseNewtonSec));
        appliedImpulse |=
            wheel->SolveLateralConstraintPart(mConstraint, -lateralMaxImpulseNewtonSec, lateralMaxImpulseNewtonSec);
    }
    return appliedImpulse;
}

} // namespace frcsim
