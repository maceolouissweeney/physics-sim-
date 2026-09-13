#include "field/field.h"

#include <cmath>
#include <stdexcept>

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include "util/errors.h"
#include "world/body_tag.h"
#include "world/layers.h"
#include "world/shapes.h"

namespace frcsim {
namespace {

constexpr float kGroundHalfExtentXY = 100.0f;
constexpr float kGroundHalfThickness = 0.5f;

bool finite(JPH::Vec3 v) {
    return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
}

} // namespace

JPH::AABox unboundedBox() {
    return JPH::AABox(JPH::Vec3::sReplicate(-1.0e6f), JPH::Vec3::sReplicate(1.0e6f));
}

Field::Field(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials), m_bounds(unboundedBox()) {}

Field::~Field() {
    clear();
}

std::uint32_t Field::addGround(float height, MaterialId material) {
    if (!std::isfinite(height)) {
        throw std::invalid_argument("ground height must be finite");
    }
    const auto shape = makeBox(JPH::Vec3(kGroundHalfExtentXY, kGroundHalfExtentXY, kGroundHalfThickness),
                               kStaticConvexRadius);
    return addBody("ground", shape.GetPtr(), JPH::Vec3(0.0f, 0.0f, height - kGroundHalfThickness),
                   JPH::Quat::sIdentity(), material);
}

std::uint32_t Field::addBox(std::string_view name, JPH::Vec3 center, JPH::Vec3 halfExtents, JPH::Quat rotation,
                            MaterialId material) {
    const auto shape = makeBox(halfExtents, kStaticConvexRadius);
    return addBody(name, shape.GetPtr(), center, rotation, material);
}

std::uint32_t Field::addCylinder(std::string_view name, JPH::Vec3 center, float radius, float halfHeight,
                                 JPH::Quat rotation, MaterialId material) {
    const auto shape = makeZCylinder(radius, halfHeight, kStaticConvexRadius);
    return addBody(name, shape.GetPtr(), center, rotation, material);
}

std::uint32_t Field::addConvexHull(std::string_view name, JPH::Vec3 center, std::span<const JPH::Vec3> points,
                                   JPH::Quat rotation, MaterialId material) {
    const auto shape = makeConvexHull(points, kStaticConvexRadius);
    return addBody(name, shape.GetPtr(), center, rotation, material);
}

std::uint32_t Field::addBody(std::string_view name, const JPH::Shape* shape, JPH::Vec3 center, JPH::Quat rotation,
                             MaterialId material) {
    if (!finite(center)) {
        throw std::invalid_argument("field primitive center must be finite");
    }
    if (rotation.IsNaN() || !rotation.IsNormalized(1.0e-3f)) {
        throw std::invalid_argument("field primitive rotation must be a unit quaternion");
    }
    const Material& properties = m_materials.get(material);
    const auto index = static_cast<std::uint32_t>(m_primitives.size());

    JPH::BodyCreationSettings settings(shape, JPH::RVec3(center), rotation.Normalized(), JPH::EMotionType::Static,
                                       ObjectLayers::kStatic);
    settings.mFriction = properties.friction;
    settings.mRestitution = properties.restitution;
    settings.mUserData = BodyTag{BodyKind::Field, material, index}.encode();

    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    JPH::Body* body = bodies.CreateBody(settings);
    if (body == nullptr) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while adding field geometry");
    }
    bodies.AddBody(body->GetID(), JPH::EActivation::DontActivate);
    m_primitives.push_back(Primitive{std::string(name), body->GetID()});
    return index;
}

void Field::setBounds(const JPH::AABox& bounds) {
    if (!bounds.IsValid() || !finite(bounds.mMin) || !finite(bounds.mMax)) {
        throw std::invalid_argument("field bounds must be finite with min <= max");
    }
    m_bounds = bounds;
}

void Field::clear() {
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (const Primitive& primitive : m_primitives) {
        bodies.RemoveBody(primitive.body);
        bodies.DestroyBody(primitive.body);
    }
    m_primitives.clear();
    m_bounds = unboundedBox();
}

const std::string& Field::primitiveName(std::uint32_t index) const {
    if (index >= m_primitives.size()) {
        throw NotFoundError("field primitive index out of range");
    }
    return m_primitives[index].name;
}

JPH::BodyID Field::primitiveBody(std::uint32_t index) const {
    if (index >= m_primitives.size()) {
        throw NotFoundError("field primitive index out of range");
    }
    return m_primitives[index].body;
}

} // namespace frcsim
