#include "world/kinematic_bodies.h"

#include <cmath>
#include <stdexcept>

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include "util/errors.h"
#include "world/body_tag.h"
#include "world/layers.h"
#include "world/shapes.h"

namespace frcsim {

KinematicBodies::KinematicBodies(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials) {}

KinematicBodies::~KinematicBodies() {
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (const JPH::BodyID id : m_bodies) {
        bodies.RemoveBody(id);
        bodies.DestroyBody(id);
    }
}

std::uint32_t KinematicBodies::addBox(JPH::Vec3 center, JPH::Vec3 halfExtents, JPH::Quat rotation,
                                      MaterialId material) {
    if (rotation.IsNaN() || !rotation.IsNormalized(1.0e-3f)) {
        throw std::invalid_argument("kinematic body rotation must be a unit quaternion");
    }
    const Material& properties = m_materials.get(material);
    const auto shape = makeBox(halfExtents, kStaticConvexRadius);
    const auto index = static_cast<std::uint32_t>(m_bodies.size());

    JPH::BodyCreationSettings settings(shape.GetPtr(), JPH::RVec3(center), rotation.Normalized(),
                                       JPH::EMotionType::Kinematic, ObjectLayers::kRobot);
    settings.mFriction = properties.friction;
    settings.mRestitution = properties.restitution;
    settings.mUserData = BodyTag{BodyKind::Kinematic, material, index}.encode();

    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    JPH::Body* body = bodies.CreateBody(settings);
    if (body == nullptr) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while adding kinematic body");
    }
    bodies.AddBody(body->GetID(), JPH::EActivation::Activate);
    m_bodies.push_back(body->GetID());
    return index;
}

void KinematicBodies::moveTo(std::uint32_t index, JPH::Vec3 position, JPH::Quat rotation, float dtSeconds) {
    if (!(dtSeconds > 0.0f) || !std::isfinite(dtSeconds)) {
        throw std::invalid_argument("dt must be finite and > 0");
    }
    if (rotation.IsNaN() || !rotation.IsNormalized(1.0e-3f)) {
        throw std::invalid_argument("kinematic body rotation must be a unit quaternion");
    }
    m_physics.GetBodyInterfaceNoLock().MoveKinematic(require(index), JPH::RVec3(position), rotation.Normalized(),
                                                     dtSeconds);
}

JPH::Vec3 KinematicBodies::position(std::uint32_t index) const {
    return JPH::Vec3(m_physics.GetBodyInterfaceNoLock().GetPosition(require(index)));
}

JPH::BodyID KinematicBodies::require(std::uint32_t index) const {
    if (index >= m_bodies.size()) {
        throw NotFoundError("kinematic body index out of range");
    }
    return m_bodies[index];
}

} // namespace frcsim
