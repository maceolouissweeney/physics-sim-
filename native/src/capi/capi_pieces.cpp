// C ABI: game piece types, spawning, state changes, and shared piece buffers.

#include <span>
#include <string>

#include "capi/capi_internal.h"

static_assert(static_cast<int>(frcsim::PieceState::OutOfBounds) == FRCSIM_PIECE_OUT_OF_BOUNDS);

using namespace frcsim::capi;

extern "C" {

void frcsim_piece_type_desc_init(frcsim_piece_type_desc* desc) {
    if (desc == nullptr) {
        return;
    }
    const frcsim::PieceTypeDesc defaults;
    *desc = frcsim_piece_type_desc{};
    desc->struct_size = sizeof(frcsim_piece_type_desc);
    desc->shape = FRCSIM_PIECE_SHAPE_SPHERE;
    desc->max_angular_velocity_rad_per_sec = defaults.maxAngularVelocityRadPerSec;
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
        d.radiusMeters = desc->radius_meters;
        d.halfHeightMeters = desc->half_height_meters;
        d.halfExtentsMeters = {desc->half_extents_meters[0], desc->half_extents_meters[1],
                               desc->half_extents_meters[2]};
        d.massKg = desc->mass_kg;
        d.maxAngularVelocityRadPerSec = desc->max_angular_velocity_rad_per_sec;
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

frcsim_status frcsim_pieces_spawn(frcsim_world* world, frcsim_piece_type_id type, const float* positions_xyz_meters,
                                 const float* velocities_xyz_meters_per_sec, uint32_t count, uint32_t* out_indices) {
    return withWorld(world, [&](frcsim_world& w) {
        if (count == 0) {
            return FRCSIM_OK;
        }
        requireNonNull(positions_xyz_meters, "positions_xyz_meters");
        const std::size_t floatCount = static_cast<std::size_t>(count) * 3;
        w.world.pieces().spawn(type, std::span<const float>(positions_xyz_meters, floatCount),
                               velocities_xyz_meters_per_sec != nullptr
                                   ? std::span<const float>(velocities_xyz_meters_per_sec, floatCount)
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

frcsim_status frcsim_piece_teleport(frcsim_world* world, uint32_t index, const float* position_xyz_meters,
                                   const float* linear_velocity_xyz_meters_per_sec,
                                   const float* angular_velocity_xyz_rad_per_sec) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.pieces().teleport(index, toVec3(position_xyz_meters, "position_xyz_meters"),
                                  toVec3OrZero(linear_velocity_xyz_meters_per_sec),
                                  toVec3OrZero(angular_velocity_xyz_rad_per_sec));
        w.refreshStats();
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_pieces_get_buffers(frcsim_world* world, frcsim_piece_buffers* out_buffers) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(out_buffers, "out_buffers");
        out_buffers->capacity = w.world.pieces().capacity();
        out_buffers->positions_xyz_meters = w.world.pieces().positionsMetersData();
        out_buffers->states = w.world.pieces().statesData();
        return FRCSIM_OK;
    });
}

} // extern "C"
