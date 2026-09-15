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

constexpr float kGroundHalfExtentXYMeters = 100.0f;
constexpr float kGroundHalfThicknessMeters = 0.5f;

bool finite(JPH::Vec3 v) {
    return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
}

} // namespace

JPH::AABox unboundedBox() {
    return JPH::AABox(JPH::Vec3::sReplicate(-1.0e6f), JPH::Vec3::sReplicate(1.0e6f));
}

Field::Field(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials), m_boundsMeters(unboundedBox()) {}

Field::~Field() {
    clear();
}

std::uint32_t Field::addGround(float heightMeters, MaterialId material) {
    if (!std::isfinite(heightMeters)) {
        throw std::invalid_argument("ground height must be finite");
    }
    const auto shape =
        makeBox(JPH::Vec3(kGroundHalfExtentXYMeters, kGroundHalfExtentXYMeters, kGroundHalfThicknessMeters),
                kStaticConvexRadiusMeters);
    return addBody("ground", shape.GetPtr(), JPH::Vec3(0.0f, 0.0f, heightMeters - kGroundHalfThicknessMeters),
                   JPH::Quat::sIdentity(), material);
}

std::uint32_t Field::addBox(std::string_view name, JPH::Vec3 centerMeters, JPH::Vec3 halfExtentsMeters,
                            JPH::Quat rotation, MaterialId material) {
    const auto shape = makeBox(halfExtentsMeters, kStaticConvexRadiusMeters);
    return addBody(name, shape.GetPtr(), centerMeters, rotation, material);
}

std::uint32_t Field::addCylinder(std::string_view name, JPH::Vec3 centerMeters, float radiusMeters,
                                 float halfHeightMeters, JPH::Quat rotation, MaterialId material) {
    const auto shape = makeZCylinder(radiusMeters, halfHeightMeters, kStaticConvexRadiusMeters);
    return addBody(name, shape.GetPtr(), centerMeters, rotation, material);
}

std::uint32_t Field::addConvexHull(std::string_view name, JPH::Vec3 centerMeters,
                                   std::span<const JPH::Vec3> pointsMeters, JPH::Quat rotation, MaterialId material) {
    const auto shape = makeConvexHull(pointsMeters, kStaticConvexRadiusMeters);
    return addBody(name, shape.GetPtr(), centerMeters, rotation, material);
}

std::uint32_t Field::addBody(std::string_view name, const JPH::Shape* shape, JPH::Vec3 centerMeters,
                             JPH::Quat rotation, MaterialId material) {
    if (!finite(centerMeters)) {
        throw std::invalid_argument("field primitive center must be finite");
    }
    if (rotation.IsNaN() || !rotation.IsNormalized(1.0e-3f)) {
        throw std::invalid_argument("field primitive rotation must be a unit quaternion");
    }
    const Material& properties = m_materials.get(material);
    const auto index = static_cast<std::uint32_t>(m_primitives.size());

    JPH::BodyCreationSettings settings(shape, JPH::RVec3(centerMeters), rotation.Normalized(),
                                       JPH::EMotionType::Static, ObjectLayers::kStatic);
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

void Field::setBoundsMeters(const JPH::AABox& boundsMeters) {
    if (!boundsMeters.IsValid() || !finite(boundsMeters.mMin) || !finite(boundsMeters.mMax)) {
        throw std::invalid_argument("field bounds must be finite with min <= max");
    }
    m_boundsMeters = boundsMeters;
}

void Field::clear() {
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    for (const Primitive& primitive : m_primitives) {
        bodies.RemoveBody(primitive.body);
        bodies.DestroyBody(primitive.body);
    }
    m_primitives.clear();
    m_boundsMeters = unboundedBox();
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
