#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "pieces/piece_type.h"
#include "world/materials.h"

namespace frcsim {

/// Values are part of the C ABI (frcsim_piece_state); do not renumber.
enum class PieceState : std::uint8_t {
    Inactive = 0,    ///< not spawned (or despawned); no body in the simulation
    OnField = 1,     ///< simulated rigid body
    Airborne = 2,    ///< simulated rigid body in flight (aero applied, Phase 4)
    InRobot = 3,     ///< held by a robot; no body in the simulation
    Scored = 4,      ///< counted by a goal; no body in the simulation
    OutOfBounds = 5, ///< left the field bounds; no body in the simulation
};
inline constexpr std::size_t kPieceStateCount = 6;

[[nodiscard]] constexpr bool isPhysical(PieceState state) {
    return state == PieceState::OnField || state == PieceState::Airborne;
}

/**
 * Structure-of-arrays registry of all game pieces, with body recycling (decision D17).
 * See docs/architecture/world-and-pieces.md. Mutate only between world steps.
 */
class PiecePool {
public:
    PiecePool(JPH::PhysicsSystem& physics, const PieceTypeRegistry& types, const MaterialTable& materials,
              std::uint32_t capacity);
    ~PiecePool();

    PiecePool(const PiecePool&) = delete;
    PiecePool& operator=(const PiecePool&) = delete;

    /// Spawns `positionsXyz.size() / 3` pieces of one type in state OnField.
    /// `velocitiesXyz` may be empty (at rest) or match positions. `outIndices` may be empty or have room
    /// for every spawned index. All-or-nothing: throws before changing anything if capacity is short.
    void spawn(PieceTypeId type, std::span<const float> positionsXyz, std::span<const float> velocitiesXyz,
               std::span<std::uint32_t> outIndices);

    /// Convenience single-piece spawn; returns the piece index.
    std::uint32_t spawnOne(PieceTypeId type, JPH::Vec3 position, JPH::Vec3 velocity = JPH::Vec3::sZero());

    /// Any spawned state -> Inactive. The index (and its body) may be reused by a later spawn of the same type.
    void despawn(std::uint32_t index);

    /// Moves a spawned piece between non-Inactive states, adding/removing its body as needed.
    /// Use despawn() to go to Inactive.
    void setState(std::uint32_t index, PieceState state);

    /// Places a spawned piece and puts it OnField (adding its body if needed).
    void teleport(std::uint32_t index, JPH::Vec3 position, JPH::Vec3 linearVelocity, JPH::Vec3 angularVelocity);

    /// Once per World::step after all substeps: refresh the output positions and mark out-of-bounds pieces.
    void postStep(const JPH::AABox& bounds);

    [[nodiscard]] std::uint32_t capacity() const { return m_capacity; }
    /// Number of indices ever used; outputs are valid for [0, highWater()).
    [[nodiscard]] std::uint32_t highWater() const { return m_highWater; }
    [[nodiscard]] std::uint32_t countInState(PieceState state) const {
        return m_stateCounts[static_cast<std::size_t>(state)];
    }

    [[nodiscard]] PieceState state(std::uint32_t index) const;
    [[nodiscard]] PieceTypeId type(std::uint32_t index) const;
    [[nodiscard]] JPH::BodyID body(std::uint32_t index) const;
    [[nodiscard]] JPH::Vec3 position(std::uint32_t index) const;

    /// Output buffers, stable for the pool's lifetime. positions: 3 floats per index; states: PieceState bytes.
    [[nodiscard]] const float* positionsData() const { return m_positions.data(); }
    [[nodiscard]] const std::uint8_t* statesData() const { return m_states.data(); }

private:
    JPH::BodyID createBody(PieceTypeId type, std::uint32_t index, JPH::Vec3 position);
    void checkSpawned(std::uint32_t index) const;
    void transition(std::uint32_t index, PieceState next);
    void writePosition(std::uint32_t index, JPH::RVec3 position);

    JPH::PhysicsSystem& m_physics;
    const PieceTypeRegistry& m_types;
    const MaterialTable& m_materials;
    std::uint32_t m_capacity;
    std::uint32_t m_highWater = 0;

    std::vector<JPH::BodyID> m_bodies;
    std::vector<PieceTypeId> m_typeOf;
    std::vector<std::uint8_t> m_states;
    std::vector<float> m_positions;
    std::vector<std::vector<std::uint32_t>> m_freeByType;
    std::array<std::uint32_t, kPieceStateCount> m_stateCounts{};
    std::vector<JPH::BodyID> m_scratchBodies;
};

} // namespace frcsim
