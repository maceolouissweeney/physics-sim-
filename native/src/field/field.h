#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "world/materials.h"

namespace frcsim {

/**
 * Static field geometry: ground, walls, and field elements, plus the playable bounds.
 * Each primitive is one static body tagged BodyKind::Field. Mutate only between world steps.
 */
class Field {
public:
    Field(JPH::PhysicsSystem& physics, const MaterialTable& materials);
    ~Field();

    Field(const Field&) = delete;
    Field& operator=(const Field&) = delete;

    /// Large ground slab whose top surface is at z = heightMeters.
    std::uint32_t addGround(float heightMeters, MaterialId material);

    std::uint32_t addBox(std::string_view name, JPH::Vec3 centerMeters, JPH::Vec3 halfExtentsMeters, JPH::Quat rotation,
                         MaterialId material);

    /// Cylinder with its axis along the local Z axis.
    std::uint32_t addCylinder(std::string_view name, JPH::Vec3 centerMeters, float radiusMeters, float halfHeightMeters,
                              JPH::Quat rotation, MaterialId material);

    /// Convex hull of `pointsMeters`, given relative to `centerMeters`.
    std::uint32_t addConvexHull(std::string_view name, JPH::Vec3 centerMeters, std::span<const JPH::Vec3> pointsMeters,
                                JPH::Quat rotation, MaterialId material);

    /// Pieces outside these bounds become OutOfBounds. Default: effectively unbounded.
    void setBoundsMeters(const JPH::AABox& boundsMeters);
    [[nodiscard]] const JPH::AABox& boundsMeters() const { return m_boundsMeters; }

    /// Removes all field bodies and resets bounds.
    void clear();

    [[nodiscard]] std::size_t size() const { return m_primitives.size(); }
    [[nodiscard]] const std::string& primitiveName(std::uint32_t index) const;
    [[nodiscard]] JPH::BodyID primitiveBody(std::uint32_t index) const;

private:
    struct Primitive {
        std::string name;
        JPH::BodyID body;
    };

    std::uint32_t addBody(std::string_view name, const JPH::Shape* shape, JPH::Vec3 centerMeters, JPH::Quat rotation,
                          MaterialId material);

    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<Primitive> m_primitives;
    JPH::AABox m_boundsMeters;
};

/// Bounds large enough to never trigger (default for worlds without field bounds).
[[nodiscard]] JPH::AABox unboundedBox();

} // namespace frcsim
