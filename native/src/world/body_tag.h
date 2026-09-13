#pragma once

#include <cstdint>

namespace frcsim {

using MaterialId = std::uint8_t;

enum class BodyKind : std::uint8_t {
    None = 0,
    Field = 1,
    Robot = 2,
    Piece = 3,
    Sensor = 4,
    Kinematic = 5,
};

/**
 * Compact identity stored in each Jolt body's 64-bit user data, so contact callbacks can identify bodies
 * without locks or lookups.
 *
 *   bits 63..60  kind
 *   bits 59..52  material id
 *   bits 51..32  reserved (zero)
 *   bits 31..0   index within the owning collection
 */
struct BodyTag {
    BodyKind kind = BodyKind::None;
    MaterialId material = 0;
    std::uint32_t index = 0;

    [[nodiscard]] constexpr std::uint64_t encode() const {
        return (static_cast<std::uint64_t>(kind) << 60) | (static_cast<std::uint64_t>(material) << 52) |
               static_cast<std::uint64_t>(index);
    }

    [[nodiscard]] static constexpr BodyTag decode(std::uint64_t value) {
        return BodyTag{static_cast<BodyKind>((value >> 60) & 0xF), static_cast<MaterialId>((value >> 52) & 0xFF),
                       static_cast<std::uint32_t>(value & 0xFFFFFFFFu)};
    }

    [[nodiscard]] static constexpr MaterialId decodeMaterial(std::uint64_t value) {
        return static_cast<MaterialId>((value >> 52) & 0xFF);
    }
};

} // namespace frcsim
