// C ABI: library, world, materials, field, kinematic bodies. Every exported function is noexcept; C++
// exceptions are translated to frcsim_status codes and never cross the ABI boundary.

#include <cstddef>
#include <string>
#include <string_view>

#include "capi/capi_internal.h"
#include "field/field_json.h"
#include "frcsim_version.h"

// The stats block is read from shared memory by bindings; its layout is ABI.
static_assert(sizeof(frcsim_world_stats) == 40);
static_assert(offsetof(frcsim_world_stats, time_seconds) == 0);
static_assert(offsetof(frcsim_world_stats, last_step_wall_seconds) == 8);
static_assert(offsetof(frcsim_world_stats, substep_count) == 16);
static_assert(offsetof(frcsim_world_stats, active_bodies) == 24);
static_assert(offsetof(frcsim_world_stats, update_error_flags) == 28);
static_assert(offsetof(frcsim_world_stats, piece_high_water) == 32);
static_assert(offsetof(frcsim_world_stats, pieces_simulated) == 36);

using namespace frcsim::capi;

namespace frcsim::capi {
namespace {
thread_local std::string t_lastError;
} // namespace

void setLastError(const char* message) noexcept {
    try {
        t_lastError = message;
    } catch (...) {
        // Could not store the message (out of memory); the status code still reports the failure.
    }
}

void clearLastError() noexcept {
    t_lastError.clear();
}

} // namespace frcsim::capi

frcsim_world::frcsim_world(const frcsim::WorldConfig& config) : world(config) {
    refreshStats();
}

void frcsim_world::refreshStats() noexcept {
    const frcsim::WorldStats& s = world.stats();
    const frcsim::PiecePool& pieces = world.pieces();
    stats.time_seconds = s.timeSeconds;
    stats.last_step_wall_seconds = s.lastStepWallSeconds;
    stats.substep_count = s.substepCount;
    stats.active_bodies = s.activeBodies;
    stats.update_error_flags = s.updateErrorFlags;
    stats.piece_high_water = pieces.highWater();
    stats.pieces_simulated =
        pieces.countInState(frcsim::PieceState::OnField) + pieces.countInState(frcsim::PieceState::Airborne);
}

namespace {

frcsim::WorldConfig toWorldConfig(const frcsim_world_config& c) {
    frcsim::WorldConfig config;
    config.maxBodies = c.max_bodies;
    config.maxBodyPairs = c.max_body_pairs;
    config.maxContactConstraints = c.max_contact_constraints;
    config.workerThreads = c.worker_threads;
    config.tempAllocatorBytes = c.temp_allocator_bytes;
    config.gravityZMetersPerSecSq = c.gravity_z_meters_per_sec_sq;
    config.maxPieces = c.max_pieces;
    config.minVelocityForRestitutionMetersPerSec = c.min_velocity_for_restitution_meters_per_sec;
    config.timeBeforeSleepSeconds = c.time_before_sleep_seconds;
    config.sleepVelocityThresholdMetersPerSec = c.sleep_velocity_threshold_meters_per_sec;
    config.solverVelocitySteps = c.solver_velocity_steps;
    config.solverPositionSteps = c.solver_position_steps;
    return config;
}

} // namespace

extern "C" {

/* ---- Library ------------------------------------------------------------------------------------- */

const char* frcsim_version_string(void) {
    return FRCSIM_VERSION_STRING;
}

uint32_t frcsim_abi_version(void) {
    return FRCSIM_ABI_VERSION;
}

const char* frcsim_status_string(frcsim_status status) {
    switch (status) {
    case FRCSIM_OK:
        return "ok";
    case FRCSIM_ERR_INVALID_ARGUMENT:
        return "invalid argument";
    case FRCSIM_ERR_OUT_OF_MEMORY:
        return "out of memory";
    case FRCSIM_ERR_CAPACITY_EXCEEDED:
        return "capacity exceeded";
    case FRCSIM_ERR_NOT_FOUND:
        return "not found";
    case FRCSIM_ERR_INTERNAL:
        return "internal error";
    }
    return "unknown status";
}

const char* frcsim_last_error(void) {
    return t_lastError.c_str();
}

/* ---- World --------------------------------------------------------------------------------------- */

void frcsim_world_config_init(frcsim_world_config* config) {
    if (config == nullptr) {
        return;
    }
    const frcsim::WorldConfig defaults;
    config->struct_size = sizeof(frcsim_world_config);
    config->max_bodies = defaults.maxBodies;
    config->max_body_pairs = defaults.maxBodyPairs;
    config->max_contact_constraints = defaults.maxContactConstraints;
    config->worker_threads = defaults.workerThreads;
    config->temp_allocator_bytes = defaults.tempAllocatorBytes;
    config->gravity_z_meters_per_sec_sq = defaults.gravityZMetersPerSecSq;
    config->max_pieces = defaults.maxPieces;
    config->min_velocity_for_restitution_meters_per_sec = defaults.minVelocityForRestitutionMetersPerSec;
    config->time_before_sleep_seconds = defaults.timeBeforeSleepSeconds;
    config->sleep_velocity_threshold_meters_per_sec = defaults.sleepVelocityThresholdMetersPerSec;
    config->solver_velocity_steps = defaults.solverVelocitySteps;
    config->solver_position_steps = defaults.solverPositionSteps;
}

frcsim_status frcsim_world_create(const frcsim_world_config* config, frcsim_world** out_world) {
    return guarded([&]() -> frcsim_status {
        if (out_world == nullptr) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "out_world must not be NULL");
        }
        *out_world = nullptr;

        frcsim_world_config effective;
        frcsim_world_config_init(&effective);
        if (config != nullptr) {
            if (config->struct_size != sizeof(frcsim_world_config)) {
                return fail(FRCSIM_ERR_INVALID_ARGUMENT,
                            "frcsim_world_config.struct_size mismatch; call frcsim_world_config_init()");
            }
            effective = *config;
        }

        *out_world = new frcsim_world(toWorldConfig(effective));
        return FRCSIM_OK;
    });
}

void frcsim_world_destroy(frcsim_world* world) {
    delete world;
}

frcsim_status frcsim_world_step(frcsim_world* world, double dt_seconds, int32_t substeps) {
    return withWorld(world, [&](frcsim_world& w) {
        w.applyRobotInputs();
        w.world.step(dt_seconds, substeps);
        w.refreshStats();
        w.refreshRobotOutputs();
        return FRCSIM_OK;
    });
}

double frcsim_world_time_seconds(const frcsim_world* world) {
    return world != nullptr ? world->world.timeSeconds() : 0.0;
}

frcsim_status frcsim_world_optimize_broadphase(frcsim_world* world) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.optimizeBroadPhase();
        return FRCSIM_OK;
    });
}

const frcsim_world_stats* frcsim_world_stats_ptr(const frcsim_world* world) {
    return world != nullptr ? &world->stats : nullptr;
}

/* ---- Materials ----------------------------------------------------------------------------------- */

frcsim_status frcsim_material_add(frcsim_world* world, const char* name, float friction, float restitution,
                                  frcsim_material_id* out_id) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(name, "name");
        const frcsim::MaterialId id = w.world.materials().add(name, frcsim::Material{friction, restitution});
        if (out_id != nullptr) {
            *out_id = id;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_material_find(frcsim_world* world, const char* name, frcsim_material_id* out_id) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(name, "name");
        requireNonNull(out_id, "out_id");
        *out_id = w.world.materials().require(name);
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_material_set_pair(frcsim_world* world, frcsim_material_id a, frcsim_material_id b,
                                       float friction, float restitution) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.materials().setPair(a, b, frcsim::Material{friction, restitution});
        return FRCSIM_OK;
    });
}

/* ---- Field --------------------------------------------------------------------------------------- */

frcsim_status frcsim_field_load_json(frcsim_world* world, const char* json_utf8, size_t length) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(json_utf8, "json_utf8");
        frcsim::loadFieldJson(w.world, std::string_view(json_utf8, length));
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_field_add_ground(frcsim_world* world, float height_meters, frcsim_material_id material,
                                     uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        writeIndex(out_index, w.world.field().addGround(height_meters, material));
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_field_add_box(frcsim_world* world, const char* name, const float* center_xyz_meters,
                                  const float* half_extents_xyz_meters, const float* rotation_xyzw,
                                  frcsim_material_id material, uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        writeIndex(out_index, w.world.field().addBox(name != nullptr ? name : "",
                                                     toVec3(center_xyz_meters, "center_xyz_meters"),
                                                     toVec3(half_extents_xyz_meters, "half_extents_xyz_meters"),
                                                     toQuatOrIdentity(rotation_xyzw), material));
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_field_set_bounds(frcsim_world* world, const float* min_xyz_meters,
                                     const float* max_xyz_meters) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.field().setBoundsMeters(
            JPH::AABox(toVec3(min_xyz_meters, "min_xyz_meters"), toVec3(max_xyz_meters, "max_xyz_meters")));
        return FRCSIM_OK;
    });
}

/* ---- Kinematic bodies ---------------------------------------------------------------------------- */

frcsim_status frcsim_kinematic_add_box(frcsim_world* world, const float* center_xyz_meters,
                                      const float* half_extents_xyz_meters, const float* rotation_xyzw,
                                      frcsim_material_id material, uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        writeIndex(out_index, w.world.kinematics().addBox(toVec3(center_xyz_meters, "center_xyz_meters"),
                                                          toVec3(half_extents_xyz_meters, "half_extents_xyz_meters"),
                                                          toQuatOrIdentity(rotation_xyzw), material));
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_kinematic_move_to(frcsim_world* world, uint32_t index, const float* position_xyz_meters,
                                      const float* rotation_xyzw, double dt_seconds) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.kinematics().moveTo(index, toVec3(position_xyz_meters, "position_xyz_meters"),
                                    toQuatOrIdentity(rotation_xyzw), static_cast<float>(dt_seconds));
        return FRCSIM_OK;
    });
}

} // extern "C"
