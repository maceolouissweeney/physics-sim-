/*
 * frcsim C ABI.
 *
 * This header is the only public native interface. The JNI bindings, and any future FFM/C++/Python
 * bindings, go through it.
 *
 * Rules:
 *  - Pure C, no C++ types. Every function is safe to call from C.
 *  - Functions never throw or abort on bad input; they return an frcsim_status. When a function fails,
 *    frcsim_last_error() returns a human-readable message for the calling thread.
 *  - Units are SI and every quantity with a unit carries it in its name (_meters, _meters_per_sec,
 *    _radians, _rad_per_sec, _volts, _amps, _ohms, _kg, _kg_meters_sq, _newton_meters, _newtons,
 *    _seconds, _hz). World frame is Z-up, matching WPILib field coordinates.
 *    Rotations are quaternions ordered x, y, z, w; NULL means identity.
 *  - A world is not thread-safe: call all functions for one world from one thread at a time.
 *  - Config structs carry struct_size so the ABI can grow; always initialize them with the matching
 *    *_init() function before setting fields.
 *  - Pointers returned for shared memory (stats, piece buffers, robot I/O) stay valid until the world is
 *    destroyed.
 */
#ifndef FRCSIM_C_H
#define FRCSIM_C_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(FRCSIM_BUILDING_LIBRARY)
#    define FRCSIM_API __declspec(dllexport)
#  else
#    define FRCSIM_API __declspec(dllimport)
#  endif
#else
#  define FRCSIM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Incremented on any breaking change to this header once released (decision D21). */
#define FRCSIM_ABI_VERSION 1

typedef enum frcsim_status {
    FRCSIM_OK = 0,
    FRCSIM_ERR_INVALID_ARGUMENT = 1,
    FRCSIM_ERR_OUT_OF_MEMORY = 2,
    FRCSIM_ERR_CAPACITY_EXCEEDED = 3,
    FRCSIM_ERR_NOT_FOUND = 4,
    FRCSIM_ERR_INTERNAL = 5
} frcsim_status;

/* ==================================================================================================== */
/* Library                                                                                              */
/* ==================================================================================================== */

/** Library semantic version, e.g. "0.1.0". Static string, never NULL. */
FRCSIM_API const char* frcsim_version_string(void);

/** ABI version the library was compiled with (compare against FRCSIM_ABI_VERSION). */
FRCSIM_API uint32_t frcsim_abi_version(void);

/** Short description of a status code. Static string, never NULL. */
FRCSIM_API const char* frcsim_status_string(frcsim_status status);

/**
 * Message describing the most recent failure on the calling thread, or "" if none.
 * The pointer stays valid until the next frcsim call on the same thread.
 */
FRCSIM_API const char* frcsim_last_error(void);

/* ==================================================================================================== */
/* World                                                                                                */
/* ==================================================================================================== */

typedef struct frcsim_world frcsim_world;

typedef struct frcsim_world_config {
    uint32_t struct_size;                           /**< set by frcsim_world_config_init */
    uint32_t max_bodies;                            /**< All bodies (statics, robots, pieces). Default 4096. */
    uint32_t max_body_pairs;                        /**< Broadphase pair capacity. Default 32768. */
    uint32_t max_contact_constraints;               /**< Contact constraint capacity. Default 16384. */
    int32_t worker_threads;                         /**< 0 = single-threaded, N > 0 = N workers (default 2), -1 = auto. */
    uint32_t temp_allocator_bytes;                  /**< Per-step scratch memory. Default 32 MiB. */
    double gravity_z_meters_per_sec_sq;             /**< Along +Z. Default -9.80665. */
    uint32_t max_pieces;                            /**< Game piece capacity, <= max_bodies. Default 1024. */
    float min_velocity_for_restitution_meters_per_sec; /**< Slower impacts don't bounce. Default 0.2. */
    float time_before_sleep_seconds;                /**< At rest this long before a body sleeps. Default 0.5. */
    float sleep_velocity_threshold_meters_per_sec;  /**< Below this a body counts as at rest. Default 0.03. */
    uint32_t solver_velocity_steps;                 /**< Default 10. */
    uint32_t solver_position_steps;                 /**< Default 2. */
} frcsim_world_config;

/** Fill a config with defaults. Always call before setting individual fields. */
FRCSIM_API void frcsim_world_config_init(frcsim_world_config* config);

/**
 * Create a world. config may be NULL to use defaults. On success *out_world receives the handle, which
 * must be released with frcsim_world_destroy(). The world starts with the standard FRC materials.
 */
FRCSIM_API frcsim_status frcsim_world_create(const frcsim_world_config* config, frcsim_world** out_world);

/** Destroy a world. NULL is ignored. */
FRCSIM_API void frcsim_world_destroy(frcsim_world* world);

/**
 * Advance the world by dt_seconds using `substeps` equal fixed sub-steps. Robot I/O inputs are read before
 * and outputs written after. Typical robot usage: dt_seconds = 0.020, substeps = 5 (4 ms physics step).
 */
FRCSIM_API frcsim_status frcsim_world_step(frcsim_world* world, double dt_seconds, int32_t substeps);

/** Simulated time since creation (0.0 if world is NULL). */
FRCSIM_API double frcsim_world_time_seconds(const frcsim_world* world);

/** Rebuild broadphase trees after adding many bodies (done automatically before the first step). */
FRCSIM_API frcsim_status frcsim_world_optimize_broadphase(frcsim_world* world);

/**
 * World statistics, refreshed by frcsim_world_step() and by piece spawn/state calls.
 * The layout is part of the ABI: bindings read it directly from shared memory.
 */
typedef struct frcsim_world_stats {
    double time_seconds;           /* offset  0 */
    double last_step_wall_seconds; /* offset  8: wall-clock duration of the last step */
    uint64_t substep_count;        /* offset 16 */
    uint32_t active_bodies;        /* offset 24: awake rigid bodies after the last step */
    uint32_t update_error_flags;   /* offset 28: union of physics update errors (capacity overflows) */
    uint32_t piece_high_water;     /* offset 32: piece outputs are valid for indices [0, piece_high_water) */
    uint32_t pieces_simulated;     /* offset 36: pieces on field or airborne */
} frcsim_world_stats;               /* size 40 */

/** Pointer to the world's stats block (NULL if world is NULL). */
FRCSIM_API const frcsim_world_stats* frcsim_world_stats_ptr(const frcsim_world* world);

/* ==================================================================================================== */
/* Materials                                                                                            */
/* ==================================================================================================== */

typedef uint8_t frcsim_material_id;
#define FRCSIM_MATERIAL_DEFAULT ((frcsim_material_id)0)

/** Add a material, or update it if the name exists. Max 64 materials per world. Coefficients are unitless. */
FRCSIM_API frcsim_status frcsim_material_add(frcsim_world* world, const char* name, float friction,
                                             float restitution, frcsim_material_id* out_id);

/** Look up a material by name; FRCSIM_ERR_NOT_FOUND if it does not exist. */
FRCSIM_API frcsim_status frcsim_material_find(frcsim_world* world, const char* name, frcsim_material_id* out_id);

/** Override the combined friction/restitution used when materials a and b touch. */
FRCSIM_API frcsim_status frcsim_material_set_pair(frcsim_world* world, frcsim_material_id a, frcsim_material_id b,
                                                  float friction, float restitution);

/* ==================================================================================================== */
/* Field                                                                                                */
/* ==================================================================================================== */

/** Load a frcsim.field JSON document (docs/reference/field-json.md). Not atomic on failure. */
FRCSIM_API frcsim_status frcsim_field_load_json(frcsim_world* world, const char* json_utf8, size_t length);

/** Add a large ground slab whose top surface is at z = height_meters. */
FRCSIM_API frcsim_status frcsim_field_add_ground(frcsim_world* world, float height_meters,
                                                frcsim_material_id material, uint32_t* out_index);

/** Add a static box. name may be NULL; rotation_xyzw may be NULL; out_index may be NULL. */
FRCSIM_API frcsim_status frcsim_field_add_box(frcsim_world* world, const char* name, const float* center_xyz_meters,
                                             const float* half_extents_xyz_meters, const float* rotation_xyzw,
                                             frcsim_material_id material, uint32_t* out_index);

/** Pieces leaving these bounds become FRCSIM_PIECE_OUT_OF_BOUNDS. */
FRCSIM_API frcsim_status frcsim_field_set_bounds(frcsim_world* world, const float* min_xyz_meters,
                                                const float* max_xyz_meters);

/* ==================================================================================================== */
/* Game pieces                                                                                          */
/* ==================================================================================================== */

typedef uint16_t frcsim_piece_type_id;

typedef enum frcsim_piece_shape {
    FRCSIM_PIECE_SHAPE_SPHERE = 0,
    FRCSIM_PIECE_SHAPE_CYLINDER = 1, /* axis along Z */
    FRCSIM_PIECE_SHAPE_BOX = 2
} frcsim_piece_shape;

typedef enum frcsim_piece_state {
    FRCSIM_PIECE_INACTIVE = 0,
    FRCSIM_PIECE_ON_FIELD = 1,
    FRCSIM_PIECE_AIRBORNE = 2,
    FRCSIM_PIECE_IN_ROBOT = 3,
    FRCSIM_PIECE_SCORED = 4,
    FRCSIM_PIECE_OUT_OF_BOUNDS = 5
} frcsim_piece_state;

typedef struct frcsim_piece_type_desc {
    uint32_t struct_size;                   /**< set by frcsim_piece_type_desc_init */
    const char* name;                       /**< unique, required */
    int32_t shape;                          /**< frcsim_piece_shape */
    float radius_meters;                    /**< sphere, cylinder */
    float half_height_meters;               /**< cylinder */
    float half_extents_meters[3];           /**< box */
    float mass_kg;                          /**< required */
    float max_angular_velocity_rad_per_sec; /**< Default 500. */
    frcsim_material_id material;            /**< Default FRCSIM_MATERIAL_DEFAULT. */
} frcsim_piece_type_desc;

FRCSIM_API void frcsim_piece_type_desc_init(frcsim_piece_type_desc* desc);

FRCSIM_API frcsim_status frcsim_piece_type_add(frcsim_world* world, const frcsim_piece_type_desc* desc,
                                              frcsim_piece_type_id* out_type);

/** FRCSIM_ERR_NOT_FOUND if no type has this name. */
FRCSIM_API frcsim_status frcsim_piece_type_find(frcsim_world* world, const char* name,
                                               frcsim_piece_type_id* out_type);

/**
 * Spawn `count` pieces on the field. positions_xyz_meters holds 3*count floats; velocities may be NULL (at
 * rest) or hold 3*count floats; out_indices may be NULL or hold count entries. All-or-nothing on capacity
 * errors.
 */
FRCSIM_API frcsim_status frcsim_pieces_spawn(frcsim_world* world, frcsim_piece_type_id type,
                                            const float* positions_xyz_meters,
                                            const float* velocities_xyz_meters_per_sec, uint32_t count,
                                            uint32_t* out_indices);

/** Return a spawned piece to FRCSIM_PIECE_INACTIVE (its index may be reused by a later spawn). */
FRCSIM_API frcsim_status frcsim_piece_despawn(frcsim_world* world, uint32_t index);

/** Move a spawned piece to another state (not INACTIVE; use frcsim_piece_despawn). */
FRCSIM_API frcsim_status frcsim_piece_set_state(frcsim_world* world, uint32_t index, int32_t state);

/** Place a spawned piece on the field. Velocity pointers may be NULL (zero). */
FRCSIM_API frcsim_status frcsim_piece_teleport(frcsim_world* world, uint32_t index, const float* position_xyz_meters,
                                              const float* linear_velocity_xyz_meters_per_sec,
                                              const float* angular_velocity_xyz_rad_per_sec);

typedef struct frcsim_piece_buffers {
    uint32_t capacity;                  /**< number of indices the buffers hold */
    const float* positions_xyz_meters;  /**< 3 * capacity floats, refreshed after each step */
    const uint8_t* states;              /**< capacity bytes of frcsim_piece_state */
} frcsim_piece_buffers;

FRCSIM_API frcsim_status frcsim_pieces_get_buffers(frcsim_world* world, frcsim_piece_buffers* out_buffers);

/* ==================================================================================================== */
/* Robots: swerve drive (docs/models/swerve.md)                                                         */
/* ==================================================================================================== */

/** DC motor parameters in WPILib DCMotor form (per motor; `count` motors share one gearbox). */
typedef struct frcsim_motor_params {
    float nominal_volts;
    float stall_torque_newton_meters;
    float stall_current_amps;
    float free_current_amps;
    float free_speed_rad_per_sec;
    int32_t count;
    float rotor_inertia_kg_meters_sq; /**< per motor */
} frcsim_motor_params;

typedef enum frcsim_motor_preset {
    FRCSIM_MOTOR_KRAKEN_X60 = 0,
    FRCSIM_MOTOR_KRAKEN_X60_FOC = 1,
    FRCSIM_MOTOR_KRAKEN_X44 = 2,
    FRCSIM_MOTOR_KRAKEN_X44_FOC = 3,
    FRCSIM_MOTOR_FALCON_500 = 4,
    FRCSIM_MOTOR_FALCON_500_FOC = 5,
    FRCSIM_MOTOR_MINION = 6
} frcsim_motor_preset;

/** Fill motor parameters from a preset (WPILib 2026 constants). */
FRCSIM_API frcsim_status frcsim_motor_params_preset(int32_t preset, int32_t count, frcsim_motor_params* out_params);

typedef enum frcsim_neutral_mode { FRCSIM_NEUTRAL_BRAKE = 0, FRCSIM_NEUTRAL_COAST = 1 } frcsim_neutral_mode;

typedef struct frcsim_swerve_module_config {
    float x_meters, y_meters;                   /**< module center in the robot frame, +X forward, +Y left */
    float wheel_radius_meters;
    float wheel_width_meters;
    float wheel_inertia_kg_meters_sq;           /**< excluding rotor */
    frcsim_motor_params drive_motor;
    float drive_gear_ratio;                     /**< motor rotations per wheel rotation */
    float drive_efficiency;
    float drive_friction_torque_newton_meters;  /**< at the wheel */
    float drive_stator_current_limit_amps;      /**< 0 = none */
    float drive_supply_current_limit_amps;      /**< 0 = none */
    int32_t drive_neutral_mode;                 /**< frcsim_neutral_mode */
    frcsim_motor_params steer_motor;
    float steer_gear_ratio;                     /**< motor rotations per module rotation */
    float steer_efficiency;
    float steer_inertia_kg_meters_sq;           /**< module about steer axis, excluding rotor */
    float steer_friction_torque_newton_meters;  /**< at the module */
    float steer_stator_current_limit_amps;
    float steer_supply_current_limit_amps;
    int32_t steer_neutral_mode;
    float coupling_gear_ratio;                  /**< drive rotor rotations per module rotation (CTRE CouplingGearRatio) */
    float tire_static_friction;
    float tire_kinetic_friction;
    float tire_transition_slip_speed_meters_per_sec;
    float scrub_radius_meters;
} frcsim_swerve_module_config;

#define FRCSIM_MAX_SWERVE_MODULES 8

typedef struct frcsim_swerve_config {
    uint32_t struct_size;                       /**< set by frcsim_swerve_config_init */
    float mass_kg;                              /**< including bumpers and battery */
    float frame_half_x_meters, frame_half_y_meters; /**< half bumper-to-bumper size */
    float bumper_bottom_meters;                 /**< above carpet */
    float bumper_height_meters;
    float com_x_meters, com_y_meters, com_height_meters;
    float yaw_inertia_kg_meters_sq;             /**< 0 = uniform box */
    frcsim_material_id bumper_material;
    float suspension_travel_meters, suspension_frequency_hz, suspension_damping_ratio;
    float battery_open_circuit_volts, battery_internal_resistance_ohms;
    float battery_brownout_volts, battery_brownout_recovery_volts;
    uint32_t module_count;
    frcsim_swerve_module_config modules[FRCSIM_MAX_SWERVE_MODULES];
    /* Sensor imperfections (0 = ideal). Noise is seeded and deterministic per platform. */
    float gyro_yaw_noise_radians;               /**< std-dev of white noise on gyro_yaw_radians */
    float gyro_yaw_drift_rate_rad_per_sec;      /**< since the last pose reset */
    float gyro_scale_error;                     /**< fractional, e.g. 0.005 */
    uint32_t drive_encoder_counts_per_rev;      /**< quantizes drive_rotor_position_radians; 0 = continuous */
    uint32_t sensor_seed;
} frcsim_swerve_config;

/**
 * Defaults: 60 kg robot, four SDS MK4i L2 modules with Kraken X60 drive and steer at
 * (+-wheel_base/2, +-track_width/2), ordered front-left, front-right, back-left, back-right.
 */
FRCSIM_API void frcsim_swerve_config_init(frcsim_swerve_config* config, float track_width_meters,
                                          float wheel_base_meters);

/**
 * Shared-memory I/O for one swerve module. Inputs are read at the start of every frcsim_world_step();
 * outputs are written at the end of it. Layout is ABI.
 */
typedef struct frcsim_swerve_module_io {
    /* inputs */
    float drive_command_volts;                /* offset  0 */
    float steer_command_volts;                /* offset  4 */
    /* outputs */
    double drive_rotor_position_radians;      /* offset  8: motor side, wheel*gear + module*coupling, quantized */
    double steer_angle_radians;               /* offset 16: continuous module angle */
    float drive_rotor_velocity_rad_per_sec;   /* offset 24: motor side, including coupling */
    float steer_velocity_rad_per_sec;         /* offset 28: module */
    float drive_applied_volts;                /* offset 32 */
    float drive_stator_current_amps;          /* offset 36 */
    float drive_supply_current_amps;          /* offset 40 */
    float steer_applied_volts;                /* offset 44 */
    float steer_stator_current_amps;          /* offset 48 */
    float steer_supply_current_amps;          /* offset 52 */
    float normal_force_newtons;               /* offset 56 */
    float slip_speed_meters_per_sec;          /* offset 60 */
} frcsim_swerve_module_io;                    /* size 64 */

typedef struct frcsim_swerve_robot_io {
    double x_meters, y_meters, z_meters;      /* offsets 0, 8, 16: robot origin, field frame */
    double yaw_radians;                       /* offset 24: continuous, true (ground truth) */
    float qx, qy, qz, qw;                     /* offsets 32..44: orientation */
    float vx_meters_per_sec, vy_meters_per_sec, vz_meters_per_sec; /* offsets 48..56: COM velocity, field frame */
    float wx_rad_per_sec, wy_rad_per_sec, wz_rad_per_sec;          /* offsets 60..68: field frame */
    float battery_volts;                      /* offset 72 */
    float battery_current_amps;               /* offset 76 */
    uint32_t brownout;                        /* offset 80: 1 while outputs are disabled */
    uint32_t module_count;                    /* offset 84 */
    frcsim_swerve_module_io modules[FRCSIM_MAX_SWERVE_MODULES]; /* offset 88 */
    double gyro_yaw_radians;                  /* offset 600: measured (scale error, drift, noise) */
} frcsim_swerve_robot_io;                     /* size 608 */

FRCSIM_API frcsim_status frcsim_robot_add_swerve(frcsim_world* world, const frcsim_swerve_config* config,
                                                float x_meters, float y_meters, float yaw_radians,
                                                uint32_t* out_robot);

/** Place a robot on the carpet at rest and re-zero its gyro to yaw_radians. */
FRCSIM_API frcsim_status frcsim_robot_reset_pose(frcsim_world* world, uint32_t robot, float x_meters, float y_meters,
                                                float yaw_radians);

/** Shared I/O block for a robot (NULL if world or index is invalid). Valid until the world is destroyed. */
FRCSIM_API frcsim_swerve_robot_io* frcsim_robot_io(frcsim_world* world, uint32_t robot);

/* ==================================================================================================== */
/* Kinematic bodies (scripted movers: test obstacles, benchmark plows)                                  */
/* ==================================================================================================== */

FRCSIM_API frcsim_status frcsim_kinematic_add_box(frcsim_world* world, const float* center_xyz_meters,
                                                 const float* half_extents_xyz_meters, const float* rotation_xyzw,
                                                 frcsim_material_id material, uint32_t* out_index);

/** Set velocity so the body reaches the pose at the end of the next step of dt_seconds. */
FRCSIM_API frcsim_status frcsim_kinematic_move_to(frcsim_world* world, uint32_t index,
                                                 const float* position_xyz_meters, const float* rotation_xyzw,
                                                 double dt_seconds);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FRCSIM_C_H */
