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

    /// Large ground slab whose top surface is at z = height.
    std::uint32_t addGround(float height, MaterialId material);

    std::uint32_t addBox(std::string_view name, JPH::Vec3 center, JPH::Vec3 halfExtents, JPH::Quat rotation,
                         MaterialId material);

    /// Cylinder with its axis along the local Z axis.
    std::uint32_t addCylinder(std::string_view name, JPH::Vec3 center, float radius, float halfHeight,
                              JPH::Quat rotation, MaterialId material);

    /// Convex hull of `points`, given relative to `center`.
    std::uint32_t addConvexHull(std::string_view name, JPH::Vec3 center, std::span<const JPH::Vec3> points,
                                JPH::Quat rotation, MaterialId material);

    /// Pieces outside these bounds become OutOfBounds. Default: effectively unbounded.
    void setBounds(const JPH::AABox& bounds);
    [[nodiscard]] const JPH::AABox& bounds() const { return m_bounds; }

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

    std::uint32_t addBody(std::string_view name, const JPH::Shape* shape, JPH::Vec3 center, JPH::Quat rotation,
                          MaterialId material);

    JPH::PhysicsSystem& m_physics;
    const MaterialTable& m_materials;
    std::vector<Primitive> m_primitives;
    JPH::AABox m_bounds;
};

/// Bounds large enough to never trigger (default for worlds without field bounds).
[[nodiscard]] JPH::AABox unboundedBox();

} // namespace frcsim
