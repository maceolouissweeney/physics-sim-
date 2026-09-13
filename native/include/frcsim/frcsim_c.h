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
 *  - Units are SI (m, kg, s, rad, V, A). World frame is Z-up, matching WPILib field coordinates.
 *    Rotations are quaternions ordered x, y, z, w; NULL means identity.
 *  - A world is not thread-safe: call all functions for one world from one thread at a time.
 *  - Config structs carry struct_size so the ABI can grow; always initialize them with the matching
 *    *_init() function before setting fields.
 *  - Pointers returned for shared memory (stats, piece buffers) stay valid until the world is destroyed.
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
    uint32_t struct_size;               /**< sizeof(frcsim_world_config); set by frcsim_world_config_init */
    uint32_t max_bodies;                /**< Capacity for all bodies (statics, robots, pieces). Default 4096. */
    uint32_t max_body_pairs;            /**< Broadphase pair capacity. Default 32768. */
    uint32_t max_contact_constraints;   /**< Contact constraint capacity. Default 16384. */
    int32_t worker_threads;             /**< 0 = single-threaded, N > 0 = N workers (default 2), -1 = auto. */
    uint32_t temp_allocator_bytes;      /**< Per-step scratch memory. Default 32 MiB. */
    double gravity_z;                   /**< m/s^2 along +Z. Default -9.80665. */
    uint32_t max_pieces;                /**< Game piece capacity, <= max_bodies. Default 1024. */
    float min_velocity_for_restitution; /**< Impacts slower than this (m/s) don't bounce. Default 0.2. */
    float time_before_sleep;            /**< Seconds at rest before a body sleeps. Default 0.5. */
    float sleep_velocity_threshold;     /**< m/s below which a body counts as at rest. Default 0.03. */
    uint32_t solver_velocity_steps;     /**< Default 10. */
    uint32_t solver_position_steps;     /**< Default 2. */
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
 * Advance the world by dt seconds using `substeps` equal fixed sub-steps.
 * Typical robot usage: dt = 0.020, substeps = 5 (4 ms physics step).
 */
FRCSIM_API frcsim_status frcsim_world_step(frcsim_world* world, double dt, int32_t substeps);

/** Simulated time in seconds since creation (0.0 if world is NULL). */
FRCSIM_API double frcsim_world_time(const frcsim_world* world);

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

/** Add a material, or update it if the name exists. Max 64 materials per world. */
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

/** Load a frcsim.field/1 JSON document (docs/reference/field-json.md). Not atomic on failure. */
FRCSIM_API frcsim_status frcsim_field_load_json(frcsim_world* world, const char* json_utf8, size_t length);

/** Add a large ground slab whose top surface is at z = height. */
FRCSIM_API frcsim_status frcsim_field_add_ground(frcsim_world* world, float height, frcsim_material_id material,
                                                uint32_t* out_index);

/** Add a static box. name may be NULL; rotation_xyzw may be NULL; out_index may be NULL. */
FRCSIM_API frcsim_status frcsim_field_add_box(frcsim_world* world, const char* name, const float* center_xyz,
                                             const float* half_extents_xyz, const float* rotation_xyzw,
                                             frcsim_material_id material, uint32_t* out_index);

/** Pieces leaving these bounds become FRCSIM_PIECE_OUT_OF_BOUNDS. */
FRCSIM_API frcsim_status frcsim_field_set_bounds(frcsim_world* world, const float* min_xyz, const float* max_xyz);

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
    uint32_t struct_size;          /**< set by frcsim_piece_type_desc_init */
    const char* name;              /**< unique, required */
    int32_t shape;                 /**< frcsim_piece_shape */
    float radius;                  /**< sphere, cylinder (m) */
    float half_height;             /**< cylinder (m) */
    float half_extents[3];         /**< box (m) */
    float mass;                    /**< kg, required */
    float max_angular_velocity;    /**< rad/s. Default 500. */
    frcsim_material_id material;   /**< Default FRCSIM_MATERIAL_DEFAULT. */
} frcsim_piece_type_desc;

FRCSIM_API void frcsim_piece_type_desc_init(frcsim_piece_type_desc* desc);

FRCSIM_API frcsim_status frcsim_piece_type_add(frcsim_world* world, const frcsim_piece_type_desc* desc,
                                              frcsim_piece_type_id* out_type);

/** FRCSIM_ERR_NOT_FOUND if no type has this name. */
FRCSIM_API frcsim_status frcsim_piece_type_find(frcsim_world* world, const char* name,
                                               frcsim_piece_type_id* out_type);

/**
 * Spawn `count` pieces on the field. positions_xyz holds 3*count floats; velocities_xyz may be NULL (at rest)
 * or hold 3*count floats; out_indices may be NULL or hold count entries. All-or-nothing on capacity errors.
 */
FRCSIM_API frcsim_status frcsim_pieces_spawn(frcsim_world* world, frcsim_piece_type_id type,
                                            const float* positions_xyz, const float* velocities_xyz, uint32_t count,
                                            uint32_t* out_indices);

/** Return a spawned piece to FRCSIM_PIECE_INACTIVE (its index may be reused by a later spawn). */
FRCSIM_API frcsim_status frcsim_piece_despawn(frcsim_world* world, uint32_t index);

/** Move a spawned piece to another state (not INACTIVE; use frcsim_piece_despawn). */
FRCSIM_API frcsim_status frcsim_piece_set_state(frcsim_world* world, uint32_t index, int32_t state);

/** Place a spawned piece on the field. Velocity pointers may be NULL (zero). */
FRCSIM_API frcsim_status frcsim_piece_teleport(frcsim_world* world, uint32_t index, const float* position_xyz,
                                              const float* linear_velocity_xyz, const float* angular_velocity_xyz);

typedef struct frcsim_piece_buffers {
    uint32_t capacity;           /**< number of indices the buffers hold */
    const float* positions_xyz;  /**< 3 * capacity floats, refreshed after each step */
    const uint8_t* states;       /**< capacity bytes of frcsim_piece_state */
} frcsim_piece_buffers;

FRCSIM_API frcsim_status frcsim_pieces_get_buffers(frcsim_world* world, frcsim_piece_buffers* out_buffers);

/* ==================================================================================================== */
/* Kinematic bodies (scripted movers: test obstacles, benchmark plows)                                  */
/* ==================================================================================================== */

FRCSIM_API frcsim_status frcsim_kinematic_add_box(frcsim_world* world, const float* center_xyz,
                                                 const float* half_extents_xyz, const float* rotation_xyzw,
                                                 frcsim_material_id material, uint32_t* out_index);

/** Set velocity so the body reaches the pose at the end of the next step of dt seconds. */
FRCSIM_API frcsim_status frcsim_kinematic_move_to(frcsim_world* world, uint32_t index, const float* position_xyz,
                                                 const float* rotation_xyzw, double dt);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FRCSIM_C_H */
