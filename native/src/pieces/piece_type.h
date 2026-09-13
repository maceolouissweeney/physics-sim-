#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/Shape.h>

#include "world/materials.h"

namespace frcsim {

using PieceTypeId = std::uint16_t;

enum class PieceShape : std::uint8_t {
    Sphere = 0,
    Cylinder = 1, ///< axis along Z
    Box = 2,
};

struct PieceTypeDesc {
    std::string name;
    PieceShape shape = PieceShape::Sphere;
    float radius = 0.0f;                    ///< sphere, cylinder
    float halfHeight = 0.0f;                ///< cylinder
    std::array<float, 3> halfExtents{};     ///< box
    float mass = 0.0f;                      ///< kg
    MaterialId material = MaterialTable::kDefault;
    float maxAngularVelocity = 500.0f;      ///< rad/s (decision D14)
};

struct PieceType {
    PieceTypeDesc desc;
    JPH::RefConst<JPH::Shape> shape;
};

/// Registered game piece types. Types are immutable once added.
class PieceTypeRegistry {
public:
    static constexpr std::size_t kMaxTypes = 32;

    /// Validates and registers a type. Throws std::invalid_argument (bad values or duplicate name),
    /// NotFoundError (unknown material) or CapacityExceededError.
    PieceTypeId add(const PieceTypeDesc& desc, const MaterialTable& materials);

    [[nodiscard]] std::optional<PieceTypeId> find(std::string_view name) const;

    /// Throws NotFoundError for an invalid id.
    [[nodiscard]] const PieceType& get(PieceTypeId id) const;

    [[nodiscard]] std::size_t size() const { return m_types.size(); }

private:
    std::vector<PieceType> m_types;
};

} // namespace frcsim
