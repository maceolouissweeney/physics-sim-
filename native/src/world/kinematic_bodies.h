#pragma once

#include <cstdint>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include "world/materials.h"

namespace frcsim {

/**
 * Scripted, infinitely massive bodies that push dynamic bodies: benchmark plows, test obstacles, moving
 * field elements. Mutate only between world steps.
 */
class KinematicBodies {
public:
    KinematicBodies(JPH::PhysicsSystem& physics, const MaterialTable& materials);
    ~KinematicBodies();

    KinematicBodies(const KinematicBodies&) = delete;
    KinematicBodies& operator=(const KinematicBodies&) = delete;

    std::uint32_t addBox(JPH::Vec3 center, JPH::Vec3 halfExtents, JPH::Quat rotation, MaterialId material);

    /// Sets the velocity so the body reaches the target pose at the end of the next step of dtSeconds.
    /// The velocity persists afterwards; call again (or with the current pose) to stop.
    void moveTo(std::uint32_t index, JPH::Vec3 position, JPH::Quat rotation, float dtSeconds);

    [[nodiscard]] JPH::Vec3 position(std::uint32_t index) const;
    [[nodiscard]] std::size_t size() const { return m_bodies.size(); }

private:
    JPH::BodyID require(std::uint32_t index) const;

    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<JPH::BodyID> m_bodies;
};

} // namespace frcsim
