#include "pieces/piece_type.h"

#include <cmath>
#include <stdexcept>

#include "util/errors.h"
#include "world/shapes.h"

namespace frcsim {
namespace {

constexpr float kPieceConvexRadius = 0.05f; // Jolt default; clamped to the shape's smallest dimension

bool positiveFinite(float value) {
    return std::isfinite(value) && value > 0.0f;
}

} // namespace

PieceTypeId PieceTypeRegistry::add(const PieceTypeDesc& desc, const MaterialTable& materials) {
    if (desc.name.empty()) {
        throw std::invalid_argument("piece type name must not be empty");
    }
    if (find(desc.name)) {
        throw std::invalid_argument("piece type '" + desc.name + "' already exists");
    }
    if (m_types.size() >= kMaxTypes) {
        throw CapacityExceededError("too many piece types (max " + std::to_string(kMaxTypes) + ")");
    }
    if (!positiveFinite(desc.mass)) {
        throw std::invalid_argument("piece type '" + desc.name + "': mass must be finite and > 0");
    }
    if (!positiveFinite(desc.maxAngularVelocity)) {
        throw std::invalid_argument("piece type '" + desc.name + "': maxAngularVelocity must be finite and > 0");
    }
    materials.get(desc.material); // throws NotFoundError for unknown ids

    PieceType type{desc, nullptr};
    switch (desc.shape) {
    case PieceShape::Sphere:
        type.shape = makeSphere(desc.radius);
        break;
    case PieceShape::Cylinder:
        type.shape = makeZCylinder(desc.radius, desc.halfHeight, kPieceConvexRadius);
        break;
    case PieceShape::Box:
        type.shape = makeBox(JPH::Vec3(desc.halfExtents[0], desc.halfExtents[1], desc.halfExtents[2]),
                             kPieceConvexRadius);
        break;
    default:
        throw std::invalid_argument("piece type '" + desc.name + "': unknown shape");
    }

    m_types.push_back(std::move(type));
    return static_cast<PieceTypeId>(m_types.size() - 1);
}

std::optional<PieceTypeId> PieceTypeRegistry::find(std::string_view name) const {
    for (std::size_t i = 0; i < m_types.size(); ++i) {
        if (m_types[i].desc.name == name) {
            return static_cast<PieceTypeId>(i);
        }
    }
    return std::nullopt;
}

const PieceType& PieceTypeRegistry::get(PieceTypeId id) const {
    if (id >= m_types.size()) {
        throw NotFoundError("unknown piece type id " + std::to_string(id));
    }
    return m_types[id];
}

} // namespace frcsim
