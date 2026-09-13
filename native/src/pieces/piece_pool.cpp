#include "pieces/piece_pool.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include "util/errors.h"
#include "world/body_tag.h"
#include "world/layers.h"

namespace frcsim {
namespace {

bool allFinite(std::span<const float> values) {
    for (float v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}

bool inside(const JPH::AABox& box, JPH::RVec3 p) {
    return p.GetX() >= box.mMin.GetX() && p.GetY() >= box.mMin.GetY() && p.GetZ() >= box.mMin.GetZ() &&
           p.GetX() <= box.mMax.GetX() && p.GetY() <= box.mMax.GetY() && p.GetZ() <= box.mMax.GetZ();
}

} // namespace

PiecePool::PiecePool(JPH::PhysicsSystem& physics, const PieceTypeRegistry& types, const MaterialTable& materials,
                     std::uint32_t capacity)
    : m_physics(physics),
      m_types(types),
      m_materials(materials),
      m_capacity(capacity),
      m_bodies(capacity),
      m_typeOf(capacity, 0),
      m_states(capacity, static_cast<std::uint8_t>(PieceState::Inactive)),
      m_positions(3u * static_cast<std::size_t>(capacity), 0.0f),
      m_freeByType(PieceTypeRegistry::kMaxTypes) {
    if (capacity == 0) {
        throw std::invalid_argument("piece capacity must be > 0");
    }
    // Reserve everything up front: spawning and recycling never reallocate.
    for (auto& freeList : m_freeByType) {
        freeList.reserve(capacity);
    }
    m_scratchBodies.reserve(capacity);
}

PiecePool::~PiecePool() {
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (std::uint32_t i = 0; i < m_highWater; ++i) {
        const JPH::BodyID id = m_bodies[i];
        if (id.IsInvalid()) {
            continue;
        }
        if (bodies.IsAdded(id)) {
            bodies.RemoveBody(id);
        }
        bodies.DestroyBody(id);
    }
}

JPH::BodyID PiecePool::createBody(PieceTypeId typeId, std::uint32_t index, JPH::Vec3 position) {
    const PieceType& type = m_types.get(typeId);
    const Material& material = m_materials.get(type.desc.material);

    JPH::BodyCreationSettings settings(type.shape.GetPtr(), JPH::RVec3(position), JPH::Quat::sIdentity(),
                                       JPH::EMotionType::Dynamic, ObjectLayers::kPiece);
    settings.mFriction = material.friction;
    settings.mRestitution = material.restitution;
    settings.mLinearDamping = 0.0f;  // decision D7
    settings.mAngularDamping = 0.0f; // decision D7
    settings.mMaxAngularVelocity = type.desc.maxAngularVelocity; // decision D14
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = type.desc.mass;
    settings.mUserData = BodyTag{BodyKind::Piece, type.desc.material, index}.encode();

    JPH::Body* body = m_physics.GetBodyInterfaceNoLock().CreateBody(settings);
    if (body == nullptr) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while spawning pieces");
    }
    return body->GetID();
}

void PiecePool::spawn(PieceTypeId typeId, std::span<const float> positionsXyz, std::span<const float> velocitiesXyz,
                      std::span<std::uint32_t> outIndices) {
    (void)m_types.get(typeId); // throws NotFoundError for an unknown id
    if (positionsXyz.size() % 3 != 0) {
        throw std::invalid_argument("positions must contain xyz triples");
    }
    const std::size_t count = positionsXyz.size() / 3;
    if (!velocitiesXyz.empty() && velocitiesXyz.size() != positionsXyz.size()) {
        throw std::invalid_argument("velocities must be empty or match positions");
    }
    if (!outIndices.empty() && outIndices.size() < count) {
        throw std::invalid_argument("outIndices is too small");
    }
    if (!allFinite(positionsXyz) || !allFinite(velocitiesXyz)) {
        throw std::invalid_argument("positions and velocities must be finite");
    }
    if (count == 0) {
        return;
    }

    // Capacity checks before touching any state (all-or-nothing).
    std::vector<std::uint32_t>& freeList = m_freeByType[typeId];
    const std::size_t reusable = freeList.size();
    const std::size_t fresh = count > reusable ? count - reusable : 0;
    if (fresh > static_cast<std::size_t>(m_capacity - m_highWater)) {
        throw CapacityExceededError("piece capacity (max_pieces = " + std::to_string(m_capacity) + ") exceeded");
    }
    if (m_physics.GetNumBodies() + fresh > m_physics.GetMaxBodies()) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while spawning pieces");
    }

    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    m_scratchBodies.clear();
    for (std::size_t k = 0; k < count; ++k) {
        const JPH::Vec3 position(positionsXyz[3 * k], positionsXyz[3 * k + 1], positionsXyz[3 * k + 2]);
        const JPH::Vec3 velocity = velocitiesXyz.empty()
                                       ? JPH::Vec3::sZero()
                                       : JPH::Vec3(velocitiesXyz[3 * k], velocitiesXyz[3 * k + 1],
                                                   velocitiesXyz[3 * k + 2]);
        std::uint32_t index;
        JPH::BodyID id;
        if (!freeList.empty()) {
            index = freeList.back();
            freeList.pop_back();
            id = m_bodies[index];
            bodies.SetPositionAndRotation(id, JPH::RVec3(position), JPH::Quat::sIdentity(),
                                          JPH::EActivation::DontActivate);
        } else {
            index = m_highWater;
            id = createBody(typeId, index, position);
            m_bodies[index] = id;
            m_typeOf[index] = typeId;
            ++m_highWater;
            ++m_stateCounts[static_cast<std::size_t>(PieceState::Inactive)]; // new index starts Inactive
        }
        bodies.SetLinearAndAngularVelocity(id, velocity, JPH::Vec3::sZero());
        transition(index, PieceState::OnField);
        writePosition(index, JPH::RVec3(position));
        m_scratchBodies.push_back(id);
        if (!outIndices.empty()) {
            outIndices[k] = index;
        }
    }

    const int n = static_cast<int>(m_scratchBodies.size());
    const JPH::BodyInterface::AddState addState = bodies.AddBodiesPrepare(m_scratchBodies.data(), n);
    bodies.AddBodiesFinalize(m_scratchBodies.data(), n, addState, JPH::EActivation::Activate);
}

std::uint32_t PiecePool::spawnOne(PieceTypeId type, JPH::Vec3 position, JPH::Vec3 velocity) {
    const float p[3] = {position.GetX(), position.GetY(), position.GetZ()};
    const float v[3] = {velocity.GetX(), velocity.GetY(), velocity.GetZ()};
    std::uint32_t index = 0;
    spawn(type, p, v, std::span<std::uint32_t>(&index, 1));
    return index;
}

void PiecePool::checkSpawned(std::uint32_t index) const {
    if (index >= m_highWater || static_cast<PieceState>(m_states[index]) == PieceState::Inactive) {
        throw NotFoundError("piece " + std::to_string(index) + " is not spawned");
    }
}

void PiecePool::despawn(std::uint32_t index) {
    checkSpawned(index);
    if (isPhysical(state(index))) {
        m_physics.GetBodyInterfaceNoLock().RemoveBody(m_bodies[index]);
    }
    transition(index, PieceState::Inactive);
    m_freeByType[m_typeOf[index]].push_back(index);
}

void PiecePool::setState(std::uint32_t index, PieceState next) {
    checkSpawned(index);
    if (next == PieceState::Inactive) {
        throw std::invalid_argument("use despawn() to make a piece Inactive");
    }
    if (static_cast<std::uint8_t>(next) >= kPieceStateCount) {
        throw std::invalid_argument("invalid piece state");
    }
    const PieceState current = state(index);
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    if (isPhysical(current) && !isPhysical(next)) {
        bodies.RemoveBody(m_bodies[index]);
    } else if (!isPhysical(current) && isPhysical(next)) {
        bodies.AddBody(m_bodies[index], JPH::EActivation::Activate);
    }
    transition(index, next);
}

void PiecePool::teleport(std::uint32_t index, JPH::Vec3 position, JPH::Vec3 linearVelocity,
                         JPH::Vec3 angularVelocity) {
    checkSpawned(index);
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    const JPH::BodyID id = m_bodies[index];
    bodies.SetPositionAndRotation(id, JPH::RVec3(position), JPH::Quat::sIdentity(), JPH::EActivation::DontActivate);
    bodies.SetLinearAndAngularVelocity(id, linearVelocity, angularVelocity);
    if (!isPhysical(state(index))) {
        bodies.AddBody(id, JPH::EActivation::Activate);
        transition(index, PieceState::OnField);
    } else {
        bodies.ActivateBody(id);
    }
    writePosition(index, JPH::RVec3(position));
}

void PiecePool::postStep(const JPH::AABox& bounds) {
    const JPH::BodyLockInterfaceNoLock& locks = m_physics.GetBodyLockInterfaceNoLock();
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (std::uint32_t i = 0; i < m_highWater; ++i) {
        if (!isPhysical(static_cast<PieceState>(m_states[i]))) {
            continue;
        }
        const JPH::Body* body = locks.TryGetBody(m_bodies[i]);
        // Sleeping bodies have not moved since their output was last written (spawn/teleport also write it).
        if (body == nullptr || !body->IsActive()) {
            continue;
        }
        const JPH::RVec3 p = body->GetPosition();
        writePosition(i, p);
        if (!inside(bounds, p)) {
            bodies.RemoveBody(m_bodies[i]);
            transition(i, PieceState::OutOfBounds);
        }
    }
}

PieceState PiecePool::state(std::uint32_t index) const {
    if (index >= m_capacity) {
        throw NotFoundError("piece index out of range");
    }
    return static_cast<PieceState>(m_states[index]);
}

PieceTypeId PiecePool::type(std::uint32_t index) const {
    checkSpawned(index);
    return m_typeOf[index];
}

JPH::BodyID PiecePool::body(std::uint32_t index) const {
    checkSpawned(index);
    return m_bodies[index];
}

JPH::Vec3 PiecePool::position(std::uint32_t index) const {
    if (index >= m_highWater) {
        throw NotFoundError("piece index out of range");
    }
    const float* p = &m_positions[3u * index];
    return JPH::Vec3(p[0], p[1], p[2]);
}

void PiecePool::transition(std::uint32_t index, PieceState next) {
    const auto current = static_cast<std::size_t>(m_states[index]);
    --m_stateCounts[current];
    ++m_stateCounts[static_cast<std::size_t>(next)];
    m_states[index] = static_cast<std::uint8_t>(next);
}

void PiecePool::writePosition(std::uint32_t index, JPH::RVec3 position) {
    float* out = &m_positions[3u * index];
    out[0] = static_cast<float>(position.GetX());
    out[1] = static_cast<float>(position.GetY());
    out[2] = static_cast<float>(position.GetZ());
}

} // namespace frcsim
