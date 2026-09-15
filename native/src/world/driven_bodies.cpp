#include "world/driven_bodies.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>

#include "util/errors.h"
#include "world/body_tag.h"
#include "world/layers.h"
#include "world/shapes.h"

namespace frcsim {
namespace {

bool positiveFinite(float value) {
    return std::isfinite(value) && value > 0.0f;
}

} // namespace

DrivenBodies::DrivenBodies(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials) {
    m_physics.AddStepListener(this);
}

DrivenBodies::~DrivenBodies() {
    m_physics.RemoveStepListener(this);
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (const Driven& driven : m_bodies) {
        bodies.RemoveBody(driven.body);
        bodies.DestroyBody(driven.body);
    }
}

std::uint32_t DrivenBodies::addBox(const BoxDesc& desc) {
    if (!positiveFinite(desc.massKg) || !positiveFinite(desc.maxForceNewtons) ||
        !positiveFinite(desc.maxTorqueNewtonMeters)) {
        throw std::invalid_argument("driven body mass, max force and max torque must be finite and > 0");
    }
    if (!std::isfinite(desc.yawRadians)) {
        throw std::invalid_argument("driven body yaw must be finite");
    }
    const Material& properties = m_materials.get(desc.material);
    const auto shape = makeBox(desc.halfExtentsMeters, kStaticConvexRadiusMeters);
    const auto index = static_cast<std::uint32_t>(m_bodies.size());

    JPH::BodyCreationSettings settings(shape.GetPtr(), JPH::RVec3(desc.centerMeters),
                                       JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), desc.yawRadians),
                                       JPH::EMotionType::Dynamic, ObjectLayers::kRobot);
    settings.mAllowedDOFs = JPH::EAllowedDOFs::Plane2D;
    settings.mGravityFactor = 0.0f;
    settings.mAllowSleeping = false;
    settings.mLinearDamping = 0.0f;
    settings.mAngularDamping = 0.0f;
    settings.mFriction = properties.friction;
    settings.mRestitution = properties.restitution;
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = desc.massKg;
    settings.mUserData = BodyTag{BodyKind::Robot, desc.material, index}.encode();

    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    JPH::Body* body = bodies.CreateBody(settings);
    if (body == nullptr) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while adding driven body");
    }
    bodies.AddBody(body->GetID(), JPH::EActivation::Activate);

    const float lengthMeters = 2.0f * desc.halfExtentsMeters.GetX();
    const float widthMeters = 2.0f * desc.halfExtentsMeters.GetY();
    m_bodies.push_back(Driven{body->GetID(), desc.massKg,
                              desc.massKg * (lengthMeters * lengthMeters + widthMeters * widthMeters) / 12.0f,
                              desc.maxForceNewtons, desc.maxTorqueNewtonMeters});
    return index;
}

void DrivenBodies::setTargetVelocity(std::uint32_t index, float vxMetersPerSec, float vyMetersPerSec,
                                     float yawRateRadPerSec) {
    if (!std::isfinite(vxMetersPerSec) || !std::isfinite(vyMetersPerSec) || !std::isfinite(yawRateRadPerSec)) {
        throw std::invalid_argument("target velocity must be finite");
    }
    require(index);
    Driven& driven = m_bodies[index];
    driven.targetVxMetersPerSec = vxMetersPerSec;
    driven.targetVyMetersPerSec = vyMetersPerSec;
    driven.targetYawRateRadPerSec = yawRateRadPerSec;
}

JPH::Vec3 DrivenBodies::positionMeters(std::uint32_t index) const {
    return JPH::Vec3(m_physics.GetBodyInterfaceNoLock().GetPosition(require(index).body));
}

JPH::Vec3 DrivenBodies::velocityMetersPerSec(std::uint32_t index) const {
    return m_physics.GetBodyInterfaceNoLock().GetLinearVelocity(require(index).body);
}

void DrivenBodies::OnStep(const JPH::PhysicsStepListenerContext& context) {
    // Runs inside PhysicsSystem::Update with all bodies locked: use the no-lock interface.
    const float dtSeconds = context.mDeltaTime;
    if (dtSeconds <= 0.0f) {
        return;
    }
    const JPH::BodyLockInterfaceNoLock& locks = m_physics.GetBodyLockInterfaceNoLock();
    for (const Driven& driven : m_bodies) {
        JPH::Body* body = locks.TryGetBody(driven.body);
        if (body == nullptr) {
            continue;
        }
        const JPH::Vec3 velocityMetersPerSec = body->GetLinearVelocity();
        JPH::Vec3 forceNewtons((driven.targetVxMetersPerSec - velocityMetersPerSec.GetX()) * driven.massKg / dtSeconds,
                               (driven.targetVyMetersPerSec - velocityMetersPerSec.GetY()) * driven.massKg / dtSeconds,
                               0.0f);
        const float magnitudeNewtons = forceNewtons.Length();
        if (magnitudeNewtons > driven.maxForceNewtons) {
            forceNewtons *= driven.maxForceNewtons / magnitudeNewtons;
        }
        body->AddForce(forceNewtons);

        const float yawRateErrorRadPerSec = driven.targetYawRateRadPerSec - body->GetAngularVelocity().GetZ();
        const float torqueNewtonMeters =
            std::clamp(yawRateErrorRadPerSec * driven.yawInertiaKgMetersSq / dtSeconds, -driven.maxTorqueNewtonMeters,
                       driven.maxTorqueNewtonMeters);
        body->AddTorque(JPH::Vec3(0.0f, 0.0f, torqueNewtonMeters));
    }
}

const DrivenBodies::Driven& DrivenBodies::require(std::uint32_t index) const {
    if (index >= m_bodies.size()) {
        throw NotFoundError("driven body index out of range");
    }
    return m_bodies[index];
}

} // namespace frcsim
