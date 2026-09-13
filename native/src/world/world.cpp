#include "world/world.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>

namespace frcsim {
namespace {

constexpr int kMaxSubsteps = 1000;
constexpr std::int32_t kMaxWorkerThreads = 64;
constexpr std::uint32_t kMaxSolverSteps = 100;

bool nonNegativeFinite(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

void validate(const WorldConfig& config) {
    if (config.maxBodies == 0) {
        throw std::invalid_argument("max_bodies must be > 0");
    }
    if (config.maxBodyPairs == 0 || config.maxContactConstraints == 0) {
        throw std::invalid_argument("max_body_pairs and max_contact_constraints must be > 0");
    }
    if (config.workerThreads < -1 || config.workerThreads > kMaxWorkerThreads) {
        throw std::invalid_argument("worker_threads must be -1 (auto), 0 (single-threaded), or 1.." +
                                    std::to_string(kMaxWorkerThreads));
    }
    if (config.tempAllocatorBytes < 1024u * 1024u) {
        throw std::invalid_argument("temp_allocator_bytes must be >= 1 MiB");
    }
    if (!std::isfinite(config.gravityZ)) {
        throw std::invalid_argument("gravity_z must be finite");
    }
    if (config.maxPieces == 0 || config.maxPieces > config.maxBodies) {
        throw std::invalid_argument("max_pieces must be in 1..max_bodies");
    }
    if (!nonNegativeFinite(config.minVelocityForRestitution) || !nonNegativeFinite(config.timeBeforeSleep) ||
        !nonNegativeFinite(config.sleepVelocityThreshold)) {
        throw std::invalid_argument("restitution/sleep thresholds must be finite and >= 0");
    }
    if (config.solverVelocitySteps < 1 || config.solverVelocitySteps > kMaxSolverSteps ||
        config.solverPositionSteps > kMaxSolverSteps) {
        throw std::invalid_argument("solver steps out of range (velocity 1..100, position 0..100)");
    }
}

std::unique_ptr<JPH::JobSystem> makeJobSystem(std::int32_t workerThreads) {
    if (workerThreads == 0) {
        return std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    }
    int threads = workerThreads;
    if (threads < 0) {
        threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
    }
    return std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads);
}

const WorldConfig& validated(const WorldConfig& config) {
    validate(config);
    return config;
}

} // namespace

World::World(const WorldConfig& config)
    : m_config(validated(config)),
      m_contactListener(m_materials),
      m_tempAllocator(std::make_unique<JPH::TempAllocatorImpl>(config.tempAllocatorBytes)),
      m_jobSystem(makeJobSystem(config.workerThreads)),
      m_physics(std::make_unique<JPH::PhysicsSystem>()) {
    m_physics->Init(config.maxBodies, 0, config.maxBodyPairs, config.maxContactConstraints, m_broadPhaseLayers,
                    m_objectVsBroadPhaseFilter, m_objectPairFilter);
    m_physics->SetGravity(JPH::Vec3(0.0f, 0.0f, static_cast<float>(config.gravityZ)));

    JPH::PhysicsSettings settings = m_physics->GetPhysicsSettings();
    settings.mMinVelocityForRestitution = config.minVelocityForRestitution;
    settings.mTimeBeforeSleep = config.timeBeforeSleep;
    settings.mPointVelocitySleepThreshold = config.sleepVelocityThreshold;
    settings.mNumVelocitySteps = config.solverVelocitySteps;
    settings.mNumPositionSteps = config.solverPositionSteps;
    m_physics->SetPhysicsSettings(settings);
    m_physics->SetContactListener(&m_contactListener);

    registerStandardFrcMaterials(m_materials);

    m_field = std::make_unique<Field>(*m_physics, m_materials);
    m_pieces = std::make_unique<PiecePool>(*m_physics, m_pieceTypes, m_materials, config.maxPieces);
    m_kinematics = std::make_unique<KinematicBodies>(*m_physics, m_materials);
    m_driven = std::make_unique<DrivenBodies>(*m_physics, m_materials);
}

World::~World() = default;

void World::optimizeBroadPhase() {
    m_physics->OptimizeBroadPhase();
    m_broadPhaseOptimized = true;
}

void World::step(double dt, int substeps) {
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        throw std::invalid_argument("dt must be finite and > 0");
    }
    if (substeps < 1 || substeps > kMaxSubsteps) {
        throw std::invalid_argument("substeps must be in 1.." + std::to_string(kMaxSubsteps));
    }
    const auto wallStart = std::chrono::steady_clock::now();

    if (!m_broadPhaseOptimized) {
        optimizeBroadPhase();
    }

    const float h = static_cast<float>(dt / substeps);
    for (int i = 0; i < substeps; ++i) {
        const JPH::EPhysicsUpdateError error = m_physics->Update(h, 1, m_tempAllocator.get(), m_jobSystem.get());
        m_stats.updateErrorFlags |= static_cast<std::uint32_t>(error);
        ++m_stats.substepCount;
    }
    m_pieces->postStep(m_field->bounds());

    m_stats.timeSeconds += dt;
    m_stats.activeBodies = m_physics->GetNumActiveBodies(JPH::EBodyType::RigidBody);
    m_stats.pieceHighWater = m_pieces->highWater();
    m_stats.piecesSimulated =
        m_pieces->countInState(PieceState::OnField) + m_pieces->countInState(PieceState::Airborne);
    m_stats.lastStepWallSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
}

} // namespace frcsim
