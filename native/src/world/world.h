#pragma once

#include <cstdint>
#include <memory>

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "field/field.h"
#include "pieces/piece_pool.h"
#include "pieces/piece_type.h"
#include "world/contact_listener.h"
#include "world/driven_bodies.h"
#include "world/jolt_runtime.h"
#include "world/kinematic_bodies.h"
#include "world/layers.h"
#include "world/materials.h"

namespace frcsim {

struct WorldConfig {
    std::uint32_t maxBodies = 4096;
    std::uint32_t maxBodyPairs = 32768;
    std::uint32_t maxContactConstraints = 16384;
    /// 0 = single-threaded job system, N > 0 = thread pool with N workers, -1 = hardware threads - 1.
    /// Default 2: fastest measured setting on a 4-core laptop (decision D24, docs/perf/phase1.md).
    std::int32_t workerThreads = 2;
    std::uint32_t tempAllocatorBytes = 32u * 1024u * 1024u;
    double gravityZ = -9.80665;
    /// Game piece capacity (must be <= maxBodies).
    std::uint32_t maxPieces = 1024;
    /// Impacts slower than this (m/s) get no bounce (decision D15).
    float minVelocityForRestitution = 0.2f;
    /// Seconds a body must stay below sleepVelocityThreshold before sleeping.
    float timeBeforeSleep = 0.5f;
    /// Point velocity (m/s) below which a body may sleep.
    float sleepVelocityThreshold = 0.03f;
    std::uint32_t solverVelocitySteps = 10;
    std::uint32_t solverPositionSteps = 2;
};

struct WorldStats {
    double timeSeconds = 0.0;
    std::uint64_t substepCount = 0;
    double lastStepWallSeconds = 0.0; ///< wall-clock duration of the most recent step()
    std::uint32_t activeBodies = 0;   ///< awake rigid bodies after the most recent step()
    std::uint32_t updateErrorFlags = 0;
    std::uint32_t pieceHighWater = 0;
    std::uint32_t piecesSimulated = 0; ///< pieces OnField or Airborne
};

/**
 * One independent simulation world: Jolt physics system, materials, field, game pieces, kinematic bodies.
 * See docs/architecture/world-and-pieces.md.
 *
 * Not thread-safe; one thread drives a world. Multiple worlds may exist concurrently.
 * Throws std::invalid_argument for bad configuration or step arguments.
 */
class World {
public:
    explicit World(const WorldConfig& config);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /// Advance by dt seconds using `substeps` equal fixed steps (one collision step each).
    void step(double dt, int substeps);

    /// Rebuild the broadphase trees; call after adding many bodies. step() does this before the first step.
    void optimizeBroadPhase();

    [[nodiscard]] double timeSeconds() const { return m_stats.timeSeconds; }
    [[nodiscard]] std::uint64_t substepCount() const { return m_stats.substepCount; }
    [[nodiscard]] std::uint32_t updateErrorFlags() const { return m_stats.updateErrorFlags; }
    [[nodiscard]] const WorldStats& stats() const { return m_stats; }

    [[nodiscard]] const WorldConfig& config() const { return m_config; }
    [[nodiscard]] JPH::PhysicsSystem& physics() { return *m_physics; }
    [[nodiscard]] const JPH::PhysicsSystem& physics() const { return *m_physics; }
    [[nodiscard]] MaterialTable& materials() { return m_materials; }
    [[nodiscard]] PieceTypeRegistry& pieceTypes() { return m_pieceTypes; }
    [[nodiscard]] Field& field() { return *m_field; }
    [[nodiscard]] PiecePool& pieces() { return *m_pieces; }
    [[nodiscard]] const PiecePool& pieces() const { return *m_pieces; }
    [[nodiscard]] KinematicBodies& kinematics() { return *m_kinematics; }
    [[nodiscard]] DrivenBodies& driven() { return *m_driven; }

private:
    // Declaration order is lifetime order (reverse destruction): the runtime comes first; the layer
    // filters, materials, contact listener and piece shapes outlive the physics system; body owners
    // (field, pieces, kinematics) are destroyed before it.
    JoltRuntime m_runtime;
    WorldConfig m_config;
    BroadPhaseLayerMap m_broadPhaseLayers;
    ObjectVsBroadPhaseFilter m_objectVsBroadPhaseFilter;
    ObjectPairFilter m_objectPairFilter;
    MaterialTable m_materials;
    MaterialContactListener m_contactListener;
    PieceTypeRegistry m_pieceTypes;
    std::unique_ptr<JPH::TempAllocator> m_tempAllocator;
    std::unique_ptr<JPH::JobSystem> m_jobSystem;
    std::unique_ptr<JPH::PhysicsSystem> m_physics;
    std::unique_ptr<Field> m_field;
    std::unique_ptr<PiecePool> m_pieces;
    std::unique_ptr<KinematicBodies> m_kinematics;
    std::unique_ptr<DrivenBodies> m_driven;

    WorldStats m_stats;
    bool m_broadPhaseOptimized = false;
};

} // namespace frcsim
