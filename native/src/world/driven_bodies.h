#pragma once

#include <cstdint>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "world/materials.h"

namespace frcsim {

/**
 * Force-limited dynamic boxes that track a target planar velocity: simple "defense bot" obstacles and
 * benchmark plows that are cheaper than a full swerve robot.
 *
 * Unlike kinematic bodies, driven bodies have finite mass and a maximum drive force (think traction
 * limit mu*m*g), so they stall against piles and walls instead of crushing through them. Each substep a
 * Jolt step listener applies F = clamp(m * (v_target - v) / h, maxForce), and a matching yaw torque.
 * Bodies move in the XY plane only (no gravity, no pitch/roll) and should be placed slightly above the
 * carpet so they do not drag on it.
 */
class DrivenBodies final : public JPH::PhysicsStepListener {
public:
    struct BoxDesc {
        JPH::Vec3 centerMeters = JPH::Vec3::sZero();
        JPH::Vec3 halfExtentsMeters = JPH::Vec3(0.45f, 0.45f, 0.08f);
        float yawRadians = 0.0f;
        float massKg = 60.0f;
        float maxForceNewtons = 590.0f;       ///< e.g. mu * m * g
        float maxTorqueNewtonMeters = 250.0f;
        MaterialId material = MaterialTable::kDefault;
    };

    DrivenBodies(JPH::PhysicsSystem& physics, const MaterialTable& materials);
    ~DrivenBodies() override;

    DrivenBodies(const DrivenBodies&) = delete;
    DrivenBodies& operator=(const DrivenBodies&) = delete;

    std::uint32_t addBox(const BoxDesc& desc);

    /// Target velocity in the field frame and yaw rate. Held until changed.
    void setTargetVelocity(std::uint32_t index, float vxMetersPerSec, float vyMetersPerSec, float yawRateRadPerSec);

    [[nodiscard]] JPH::Vec3 positionMeters(std::uint32_t index) const;
    [[nodiscard]] JPH::Vec3 velocityMetersPerSec(std::uint32_t index) const;
    [[nodiscard]] std::size_t size() const { return m_bodies.size(); }

    void OnStep(const JPH::PhysicsStepListenerContext& context) override;

private:
    struct Driven {
        JPH::BodyID body;
        float massKg;
        float yawInertiaKgMetersSq;
        float maxForceNewtons;
        float maxTorqueNewtonMeters;
        float targetVxMetersPerSec = 0.0f;
        float targetVyMetersPerSec = 0.0f;
        float targetYawRateRadPerSec = 0.0f;
    };

    const Driven& require(std::uint32_t index) const;

    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<Driven> m_bodies;
};

} // namespace frcsim
