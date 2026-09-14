// Implementation of the C ABI declared in <frcsim/frcsim_c.h>.
//
// Every exported function is noexcept: C++ exceptions are translated to frcsim_status codes here and
// never cross the ABI boundary.

#include <frcsim/frcsim_c.h>

#include <cstddef>
#include <exception>
#include <memory>
#include <new>
#include <vector>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include <Jolt/Jolt.h>

#include "field/field_json.h"
#include "frcsim_version.h"
#include "util/errors.h"
#include "world/world.h"

// The stats block is read from shared memory by bindings; its layout is ABI.
static_assert(sizeof(frcsim_world_stats) == 40);
static_assert(offsetof(frcsim_world_stats, time_seconds) == 0);
static_assert(offsetof(frcsim_world_stats, last_step_wall_seconds) == 8);
static_assert(offsetof(frcsim_world_stats, substep_count) == 16);
static_assert(offsetof(frcsim_world_stats, active_bodies) == 24);
static_assert(offsetof(frcsim_world_stats, update_error_flags) == 28);
static_assert(offsetof(frcsim_world_stats, piece_high_water) == 32);
static_assert(offsetof(frcsim_world_stats, pieces_simulated) == 36);
static_assert(static_cast<int>(frcsim::PieceState::OutOfBounds) == FRCSIM_PIECE_OUT_OF_BOUNDS);
static_assert(sizeof(frcsim_swerve_module_io) == 64);
static_assert(offsetof(frcsim_swerve_module_io, drive_rotor_position) == 8);
static_assert(offsetof(frcsim_swerve_module_io, slip_speed) == 60);
static_assert(sizeof(frcsim_swerve_robot_io) == 608);
static_assert(offsetof(frcsim_swerve_robot_io, gyro_yaw) == 600);
static_assert(offsetof(frcsim_swerve_robot_io, yaw) == 24);
static_assert(offsetof(frcsim_swerve_robot_io, qx) == 32);
static_assert(offsetof(frcsim_swerve_robot_io, battery_voltage) == 72);
static_assert(offsetof(frcsim_swerve_robot_io, module_count) == 84);
static_assert(offsetof(frcsim_swerve_robot_io, modules) == 88);
static_assert(frcsim::kMaxSwerveModules == FRCSIM_MAX_SWERVE_MODULES);

struct frcsim_world {
    explicit frcsim_world(const frcsim::WorldConfig& config) : world(config) { refreshStats(); }

    /// Copies module voltage inputs from shared memory into the robots.
    void applyRobotInputs() {
        for (std::size_t r = 0; r < robotIo.size(); ++r) {
            frcsim::SwerveRobot& robot = world.robots().swerve(static_cast<std::uint32_t>(r));
            for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
                const frcsim_swerve_module_io& io = robotIo[r]->modules[m];
                robot.setModuleVoltages(m, io.drive_voltage, io.steer_voltage);
            }
        }
    }

    /// Writes robot outputs into shared memory.
    void refreshRobotOutputs() noexcept {
        for (std::size_t r = 0; r < robotIo.size(); ++r) {
            const frcsim::SwerveRobot& robot = world.robots().swerve(static_cast<std::uint32_t>(r));
            frcsim_swerve_robot_io& io = *robotIo[r];
            const frcsim::RobotPose pose = robot.pose();
            io.x = pose.position.GetX();
            io.y = pose.position.GetY();
            io.z = pose.position.GetZ();
            io.yaw = robot.continuousYaw();
            io.gyro_yaw = robot.measuredGyroYaw();
            io.qx = pose.rotation.GetX();
            io.qy = pose.rotation.GetY();
            io.qz = pose.rotation.GetZ();
            io.qw = pose.rotation.GetW();
            io.vx = pose.linearVelocity.GetX();
            io.vy = pose.linearVelocity.GetY();
            io.vz = pose.linearVelocity.GetZ();
            io.wx = pose.angularVelocity.GetX();
            io.wy = pose.angularVelocity.GetY();
            io.wz = pose.angularVelocity.GetZ();
            io.battery_voltage = robot.battery().voltage();
            io.battery_current = robot.battery().current();
            io.brownout = robot.battery().brownout() ? 1u : 0u;
            io.module_count = static_cast<std::uint32_t>(robot.moduleCount());
            for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
                const frcsim::SwerveModuleState& s = robot.module(m);
                const frcsim::SwerveModuleConfig& c = robot.config().modules[m];
                frcsim_swerve_module_io& out = io.modules[m];
                out.drive_rotor_position = robot.measuredDriveRotorPosition(m);
                out.steer_angle = s.steerAngle;
                out.drive_rotor_velocity = s.wheelVelocity * c.driveGearRatio;
                out.steer_velocity = s.steerVelocity;
                out.drive_applied_voltage = s.driveAppliedVoltage;
                out.drive_stator_current = s.driveStatorCurrent;
                out.drive_supply_current = s.driveSupplyCurrent;
                out.steer_applied_voltage = s.steerAppliedVoltage;
                out.steer_stator_current = s.steerStatorCurrent;
                out.steer_supply_current = s.steerSupplyCurrent;
                out.normal_force = s.normalForce;
                out.slip_speed = s.slipSpeed;
            }
        }
    }

    void refreshStats() noexcept {
        const frcsim::WorldStats& s = world.stats();
        const frcsim::PiecePool& pieces = world.pieces();
        stats.time_seconds = s.timeSeconds;
        stats.last_step_wall_seconds = s.lastStepWallSeconds;
        stats.substep_count = s.substepCount;
        stats.active_bodies = s.activeBodies;
        stats.update_error_flags = s.updateErrorFlags;
        stats.piece_high_water = pieces.highWater();
        stats.pieces_simulated = pieces.countInState(frcsim::PieceState::OnField) +
                                 pieces.countInState(frcsim::PieceState::Airborne);
    }

    frcsim::World world;
    frcsim_world_stats stats{};
    std::vector<std::unique_ptr<frcsim_swerve_robot_io>> robotIo; // stable addresses for shared memory
};

namespace {

thread_local std::string t_lastError;

frcsim_status fail(frcsim_status status, const char* message) noexcept {
    try {
        t_lastError = message;
    } catch (...) {
        // Could not store the message (out of memory); the status code still reports the failure.
    }
    return status;
}

/// Runs body(), mapping exceptions to status codes. body must return frcsim_status.
template <typename Fn>
frcsim_status guarded(Fn&& body) noexcept {
    try {
        t_lastError.clear();
        return body();
    } catch (const std::invalid_argument& e) {
        return fail(FRCSIM_ERR_INVALID_ARGUMENT, e.what());
    } catch (const frcsim::CapacityExceededError& e) {
        return fail(FRCSIM_ERR_CAPACITY_EXCEEDED, e.what());
    } catch (const frcsim::NotFoundError& e) {
        return fail(FRCSIM_ERR_NOT_FOUND, e.what());
    } catch (const std::bad_alloc&) {
        return fail(FRCSIM_ERR_OUT_OF_MEMORY, "out of memory");
    } catch (const std::exception& e) {
        return fail(FRCSIM_ERR_INTERNAL, e.what());
    } catch (...) {
        return fail(FRCSIM_ERR_INTERNAL, "unknown native exception");
    }
}

/// Like guarded(), but first validates the world handle.
template <typename Fn>
frcsim_status withWorld(frcsim_world* world, Fn&& body) noexcept {
    return guarded([&]() -> frcsim_status {
        if (world == nullptr) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "world must not be NULL");
        }
        return body(*world);
    });
}

void requireNonNull(const void* pointer, const char* name) {
    if (pointer == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be NULL");
    }
}

JPH::Vec3 toVec3(const float* xyz, const char* name) {
    requireNonNull(xyz, name);
    return JPH::Vec3(xyz[0], xyz[1], xyz[2]);
}

JPH::Vec3 toVec3OrZero(const float* xyz) {
    return xyz != nullptr ? JPH::Vec3(xyz[0], xyz[1], xyz[2]) : JPH::Vec3::sZero();
}

JPH::Quat toQuatOrIdentity(const float* xyzw) {
    return xyzw != nullptr ? JPH::Quat(xyzw[0], xyzw[1], xyzw[2], xyzw[3]) : JPH::Quat::sIdentity();
}

frcsim::WorldConfig toWorldConfig(const frcsim_world_config& c) {
    frcsim::WorldConfig config;
    config.maxBodies = c.max_bodies;
    config.maxBodyPairs = c.max_body_pairs;
    config.maxContactConstraints = c.max_contact_constraints;
    config.workerThreads = c.worker_threads;
    config.tempAllocatorBytes = c.temp_allocator_bytes;
    config.gravityZ = c.gravity_z;
    config.maxPieces = c.max_pieces;
    config.minVelocityForRestitution = c.min_velocity_for_restitution;
    config.timeBeforeSleep = c.time_before_sleep;
    config.sleepVelocityThreshold = c.sleep_velocity_threshold;
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
    config->gravity_z = defaults.gravityZ;
    config->max_pieces = defaults.maxPieces;
    config->min_velocity_for_restitution = defaults.minVelocityForRestitution;
    config->time_before_sleep = defaults.timeBeforeSleep;
    config->sleep_velocity_threshold = defaults.sleepVelocityThreshold;
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

frcsim_status frcsim_world_step(frcsim_world* world, double dt, int32_t substeps) {
    return withWorld(world, [&](frcsim_world& w) {
        w.applyRobotInputs();
        w.world.step(dt, substeps);
        w.refreshStats();
        w.refreshRobotOutputs();
        return FRCSIM_OK;
    });
}

double frcsim_world_time(const frcsim_world* world) {
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

frcsim_status frcsim_field_add_ground(frcsim_world* world, float height, frcsim_material_id material,
                                     uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        const std::uint32_t index = w.world.field().addGround(height, material);
        if (out_index != nullptr) {
            *out_index = index;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_field_add_box(frcsim_world* world, const char* name, const float* center_xyz,
                                  const float* half_extents_xyz, const float* rotation_xyzw,
                                  frcsim_material_id material, uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        const std::uint32_t index =
            w.world.field().addBox(name != nullptr ? name : "", toVec3(center_xyz, "center_xyz"),
                                   toVec3(half_extents_xyz, "half_extents_xyz"), toQuatOrIdentity(rotation_xyzw),
                                   material);
        if (out_index != nullptr) {
            *out_index = index;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_field_set_bounds(frcsim_world* world, const float* min_xyz, const float* max_xyz) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.field().setBounds(JPH::AABox(toVec3(min_xyz, "min_xyz"), toVec3(max_xyz, "max_xyz")));
        return FRCSIM_OK;
    });
}

/* ---- Game pieces --------------------------------------------------------------------------------- */

void frcsim_piece_type_desc_init(frcsim_piece_type_desc* desc) {
    if (desc == nullptr) {
        return;
    }
    const frcsim::PieceTypeDesc defaults;
    *desc = frcsim_piece_type_desc{};
    desc->struct_size = sizeof(frcsim_piece_type_desc);
    desc->shape = FRCSIM_PIECE_SHAPE_SPHERE;
    desc->max_angular_velocity = defaults.maxAngularVelocity;
    desc->material = FRCSIM_MATERIAL_DEFAULT;
}

frcsim_status frcsim_piece_type_add(frcsim_world* world, const frcsim_piece_type_desc* desc,
                                   frcsim_piece_type_id* out_type) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(desc, "desc");
        if (desc->struct_size != sizeof(frcsim_piece_type_desc)) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT,
                        "frcsim_piece_type_desc.struct_size mismatch; call frcsim_piece_type_desc_init()");
        }
        requireNonNull(desc->name, "desc->name");
        if (desc->shape < FRCSIM_PIECE_SHAPE_SPHERE || desc->shape > FRCSIM_PIECE_SHAPE_BOX) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "desc->shape is not a frcsim_piece_shape");
        }
        frcsim::PieceTypeDesc d;
        d.name = desc->name;
        d.shape = static_cast<frcsim::PieceShape>(desc->shape);
        d.radius = desc->radius;
        d.halfHeight = desc->half_height;
        d.halfExtents = {desc->half_extents[0], desc->half_extents[1], desc->half_extents[2]};
        d.mass = desc->mass;
        d.maxAngularVelocity = desc->max_angular_velocity;
        d.material = desc->material;
        const frcsim::PieceTypeId id = w.world.pieceTypes().add(d, w.world.materials());
        if (out_type != nullptr) {
            *out_type = id;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_piece_type_find(frcsim_world* world, const char* name, frcsim_piece_type_id* out_type) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(name, "name");
        requireNonNull(out_type, "out_type");
        const auto id = w.world.pieceTypes().find(name);
        if (!id) {
            return fail(FRCSIM_ERR_NOT_FOUND, (std::string("unknown piece type '") + name + "'").c_str());
        }
        *out_type = *id;
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_pieces_spawn(frcsim_world* world, frcsim_piece_type_id type, const float* positions_xyz,
                                 const float* velocities_xyz, uint32_t count, uint32_t* out_indices) {
    return withWorld(world, [&](frcsim_world& w) {
        if (count == 0) {
            return FRCSIM_OK;
        }
        requireNonNull(positions_xyz, "positions_xyz");
        const std::size_t n = static_cast<std::size_t>(count) * 3;
        w.world.pieces().spawn(type, std::span<const float>(positions_xyz, n),
                               velocities_xyz != nullptr ? std::span<const float>(velocities_xyz, n)
                                                         : std::span<const float>(),
                               out_indices != nullptr ? std::span<std::uint32_t>(out_indices, count)
                                                      : std::span<std::uint32_t>());
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_piece_despawn(frcsim_world* world, uint32_t index) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.pieces().despawn(index);
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_piece_set_state(frcsim_world* world, uint32_t index, int32_t state) {
    return withWorld(world, [&](frcsim_world& w) {
        if (state < FRCSIM_PIECE_INACTIVE || state > FRCSIM_PIECE_OUT_OF_BOUNDS) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "state is not a frcsim_piece_state");
        }
        w.world.pieces().setState(index, static_cast<frcsim::PieceState>(state));
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_piece_teleport(frcsim_world* world, uint32_t index, const float* position_xyz,
                                   const float* linear_velocity_xyz, const float* angular_velocity_xyz) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.pieces().teleport(index, toVec3(position_xyz, "position_xyz"), toVec3OrZero(linear_velocity_xyz),
                                  toVec3OrZero(angular_velocity_xyz));
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_pieces_get_buffers(frcsim_world* world, frcsim_piece_buffers* out_buffers) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(out_buffers, "out_buffers");
        out_buffers->capacity = w.world.pieces().capacity();
        out_buffers->positions_xyz = w.world.pieces().positionsData();
        out_buffers->states = w.world.pieces().statesData();
        return FRCSIM_OK;
    });
}

/* ---- Robots -------------------------------------------------------------------------------------- */

frcsim_status frcsim_motor_params_preset(int32_t preset, int32_t count, frcsim_motor_params* out_params) {
    return guarded([&]() -> frcsim_status {
        requireNonNull(out_params, "out_params");
        if (count < 1) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "motor count must be >= 1");
        }
        frcsim::DcMotorParams p;
        switch (preset) {
        case FRCSIM_MOTOR_KRAKEN_X60: p = frcsim::DcMotorParams::krakenX60(count); break;
        case FRCSIM_MOTOR_KRAKEN_X60_FOC: p = frcsim::DcMotorParams::krakenX60Foc(count); break;
        case FRCSIM_MOTOR_KRAKEN_X44: p = frcsim::DcMotorParams::krakenX44(count); break;
        case FRCSIM_MOTOR_KRAKEN_X44_FOC: p = frcsim::DcMotorParams::krakenX44Foc(count); break;
        case FRCSIM_MOTOR_FALCON_500: p = frcsim::DcMotorParams::falcon500(count); break;
        case FRCSIM_MOTOR_FALCON_500_FOC: p = frcsim::DcMotorParams::falcon500Foc(count); break;
        case FRCSIM_MOTOR_NEO: p = frcsim::DcMotorParams::neo(count); break;
        case FRCSIM_MOTOR_NEO_VORTEX: p = frcsim::DcMotorParams::neoVortex(count); break;
        default: return fail(FRCSIM_ERR_INVALID_ARGUMENT, "unknown motor preset");
        }
        *out_params = frcsim_motor_params{p.nominalVoltage, p.stallTorque, p.stallCurrent, p.freeCurrent,
                                          p.freeSpeed,      p.count,       p.rotorInertia};
        return FRCSIM_OK;
    });
}

void frcsim_swerve_config_init(frcsim_swerve_config* config, float track_width, float wheel_base) {
    if (config == nullptr) {
        return;
    }
    const frcsim::SwerveDriveConfig d = frcsim::makeRectangularSwerve(track_width, wheel_base);
    *config = frcsim_swerve_config{};
    config->struct_size = sizeof(frcsim_swerve_config);
    config->mass = d.mass;
    config->frame_half_x = d.frameHalfX;
    config->frame_half_y = d.frameHalfY;
    config->bumper_bottom = d.bumperBottom;
    config->bumper_height = d.bumperHeight;
    config->com_x = d.comX;
    config->com_y = d.comY;
    config->com_height = d.comHeight;
    config->yaw_inertia = d.yawInertia;
    config->bumper_material = FRCSIM_MATERIAL_DEFAULT;
    config->suspension_travel = d.suspension.travel;
    config->suspension_frequency = d.suspension.frequency;
    config->suspension_damping_ratio = d.suspension.dampingRatio;
    config->battery_open_circuit_voltage = d.battery.openCircuitVoltage;
    config->battery_internal_resistance = d.battery.internalResistance;
    config->battery_brownout_voltage = d.battery.brownoutVoltage;
    config->battery_brownout_recovery_voltage = d.battery.brownoutRecoveryVoltage;
    config->module_count = static_cast<uint32_t>(d.modules.size());
    config->gyro_yaw_noise = d.sensors.gyroYawNoise;
    config->gyro_yaw_drift_rate = d.sensors.gyroYawDriftRate;
    config->gyro_scale_error = d.sensors.gyroScaleError;
    config->drive_encoder_counts_per_rev = d.sensors.driveEncoderCountsPerRev;
    config->sensor_seed = d.sensors.seed;
    const auto toC = [](const frcsim::DcMotorParams& p) {
        return frcsim_motor_params{p.nominalVoltage, p.stallTorque, p.stallCurrent, p.freeCurrent,
                                   p.freeSpeed,      p.count,       p.rotorInertia};
    };
    for (std::size_t i = 0; i < d.modules.size(); ++i) {
        const frcsim::SwerveModuleConfig& m = d.modules[i];
        frcsim_swerve_module_config& c = config->modules[i];
        c.x = m.x;
        c.y = m.y;
        c.wheel_radius = m.wheelRadius;
        c.wheel_width = m.wheelWidth;
        c.wheel_inertia = m.wheelInertia;
        c.drive_motor = toC(m.driveMotor);
        c.drive_gear_ratio = m.driveGearRatio;
        c.drive_efficiency = m.driveEfficiency;
        c.drive_friction_torque = m.driveFrictionTorque;
        c.drive_stator_current_limit = m.driveCurrentLimits.stator;
        c.drive_supply_current_limit = m.driveCurrentLimits.supply;
        c.drive_neutral_mode = static_cast<int32_t>(m.driveNeutralMode);
        c.steer_motor = toC(m.steerMotor);
        c.steer_gear_ratio = m.steerGearRatio;
        c.steer_efficiency = m.steerEfficiency;
        c.steer_inertia = m.steerInertia;
        c.steer_friction_torque = m.steerFrictionTorque;
        c.steer_stator_current_limit = m.steerCurrentLimits.stator;
        c.steer_supply_current_limit = m.steerCurrentLimits.supply;
        c.steer_neutral_mode = static_cast<int32_t>(m.steerNeutralMode);
        c.tire_static_friction = m.tire.staticFriction;
        c.tire_kinetic_friction = m.tire.kineticFriction;
        c.tire_transition_slip_speed = m.tire.transitionSlipSpeed;
        c.scrub_radius = m.scrubRadius;
    }
}

frcsim_status frcsim_robot_add_swerve(frcsim_world* world, const frcsim_swerve_config* config, float x, float y,
                                     float yaw, uint32_t* out_robot) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(config, "config");
        if (config->struct_size != sizeof(frcsim_swerve_config)) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT,
                        "frcsim_swerve_config.struct_size mismatch; call frcsim_swerve_config_init()");
        }
        if (config->module_count > FRCSIM_MAX_SWERVE_MODULES) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "module_count exceeds FRCSIM_MAX_SWERVE_MODULES");
        }
        const auto neutral = [](int32_t mode) {
            if (mode != FRCSIM_NEUTRAL_BRAKE && mode != FRCSIM_NEUTRAL_COAST) {
                throw std::invalid_argument("neutral mode must be FRCSIM_NEUTRAL_BRAKE or FRCSIM_NEUTRAL_COAST");
            }
            return static_cast<frcsim::NeutralMode>(mode);
        };
        const auto toCpp = [](const frcsim_motor_params& p) {
            frcsim::DcMotorParams m;
            m.nominalVoltage = p.nominal_voltage;
            m.stallTorque = p.stall_torque;
            m.stallCurrent = p.stall_current;
            m.freeCurrent = p.free_current;
            m.freeSpeed = p.free_speed;
            m.count = p.count;
            m.rotorInertia = p.rotor_inertia;
            return m;
        };

        frcsim::SwerveDriveConfig d;
        d.mass = config->mass;
        d.frameHalfX = config->frame_half_x;
        d.frameHalfY = config->frame_half_y;
        d.bumperBottom = config->bumper_bottom;
        d.bumperHeight = config->bumper_height;
        d.comX = config->com_x;
        d.comY = config->com_y;
        d.comHeight = config->com_height;
        d.yawInertia = config->yaw_inertia;
        d.bumperMaterial = config->bumper_material;
        d.suspension = {config->suspension_travel, config->suspension_frequency, config->suspension_damping_ratio};
        d.battery = {config->battery_open_circuit_voltage, config->battery_internal_resistance,
                     config->battery_brownout_voltage, config->battery_brownout_recovery_voltage};
        d.sensors = {config->gyro_yaw_noise, config->gyro_yaw_drift_rate, config->gyro_scale_error,
                     config->drive_encoder_counts_per_rev, config->sensor_seed};
        for (uint32_t i = 0; i < config->module_count; ++i) {
            const frcsim_swerve_module_config& c = config->modules[i];
            frcsim::SwerveModuleConfig m;
            m.x = c.x;
            m.y = c.y;
            m.wheelRadius = c.wheel_radius;
            m.wheelWidth = c.wheel_width;
            m.wheelInertia = c.wheel_inertia;
            m.driveMotor = toCpp(c.drive_motor);
            m.driveGearRatio = c.drive_gear_ratio;
            m.driveEfficiency = c.drive_efficiency;
            m.driveFrictionTorque = c.drive_friction_torque;
            m.driveCurrentLimits = {c.drive_stator_current_limit, c.drive_supply_current_limit};
            m.driveNeutralMode = neutral(c.drive_neutral_mode);
            m.steerMotor = toCpp(c.steer_motor);
            m.steerGearRatio = c.steer_gear_ratio;
            m.steerEfficiency = c.steer_efficiency;
            m.steerInertia = c.steer_inertia;
            m.steerFrictionTorque = c.steer_friction_torque;
            m.steerCurrentLimits = {c.steer_stator_current_limit, c.steer_supply_current_limit};
            m.steerNeutralMode = neutral(c.steer_neutral_mode);
            m.tire = {c.tire_static_friction, c.tire_kinetic_friction, c.tire_transition_slip_speed};
            m.scrubRadius = c.scrub_radius;
            d.modules.push_back(m);
        }

        w.robotIo.reserve(w.robotIo.size() + 1); // reserve first so push_back below cannot throw after adding
        const std::uint32_t index = w.world.robots().addSwerve(d, x, y, yaw);
        w.robotIo.push_back(std::make_unique<frcsim_swerve_robot_io>());
        w.refreshRobotOutputs();
        if (out_robot != nullptr) {
            *out_robot = index;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_robot_reset_pose(frcsim_world* world, uint32_t robot, float x, float y, float yaw) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.robots().swerve(robot).resetPose(x, y, yaw);
        w.refreshRobotOutputs();
        return FRCSIM_OK;
    });
}

frcsim_swerve_robot_io* frcsim_robot_io(frcsim_world* world, uint32_t robot) {
    if (world == nullptr || robot >= world->robotIo.size()) {
        return nullptr;
    }
    return world->robotIo[robot].get();
}

/* ---- Kinematic bodies ---------------------------------------------------------------------------- */

frcsim_status frcsim_kinematic_add_box(frcsim_world* world, const float* center_xyz, const float* half_extents_xyz,
                                      const float* rotation_xyzw, frcsim_material_id material,
                                      uint32_t* out_index) {
    return withWorld(world, [&](frcsim_world& w) {
        const std::uint32_t index =
            w.world.kinematics().addBox(toVec3(center_xyz, "center_xyz"), toVec3(half_extents_xyz, "half_extents_xyz"),
                                        toQuatOrIdentity(rotation_xyzw), material);
        if (out_index != nullptr) {
            *out_index = index;
        }
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_kinematic_move_to(frcsim_world* world, uint32_t index, const float* position_xyz,
                                      const float* rotation_xyzw, double dt) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.kinematics().moveTo(index, toVec3(position_xyz, "position_xyz"), toQuatOrIdentity(rotation_xyzw),
                                    static_cast<float>(dt));
        return FRCSIM_OK;
    });
}

} // extern "C"
