// JNI bindings for org.frcsim.jni.FrcSimJNI: game piece types, spawning, and shared piece buffers.

#include "jni/jni_support.h"

using namespace frcsim::jni;

extern "C" {

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceTypeAdd(
    JNIEnv* env, jclass, jlong handle, jstring name, jint shape, jfloat radiusMeters, jfloat halfHeightMeters,
    jfloat halfXMeters, jfloat halfYMeters, jfloat halfZMeters, jfloat massKg, jint material,
    jfloat maxAngularVelocityRadPerSec) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    frcsim_piece_type_desc desc;
    frcsim_piece_type_desc_init(&desc);
    desc.name = n.get();
    desc.shape = shape;
    desc.radius_meters = radiusMeters;
    desc.half_height_meters = halfHeightMeters;
    desc.half_extents_meters[0] = halfXMeters;
    desc.half_extents_meters[1] = halfYMeters;
    desc.half_extents_meters[2] = halfZMeters;
    desc.mass_kg = massKg;
    desc.material = static_cast<frcsim_material_id>(material);
    desc.max_angular_velocity_rad_per_sec = maxAngularVelocityRadPerSec;
    frcsim_piece_type_id id = 0;
    return toJavaIndex(env, frcsim_piece_type_add(world, &desc, &id), id);
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceTypeFind(JNIEnv* env, jclass, jlong handle, jstring name) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    frcsim_piece_type_id id = 0;
    const frcsim_status status = frcsim_piece_type_find(world, n.get(), &id);
    if (status == FRCSIM_ERR_NOT_FOUND) {
        return -1;
    }
    return toJavaIndex(env, status, id);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_piecesSpawn(JNIEnv* env, jclass, jlong handle, jint type,
                                                                  jfloatArray positionsXyzMeters,
                                                                  jfloatArray velocitiesXyzMetersPerSec,
                                                                  jintArray outIndices) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return;
    }
    if (positionsXyzMeters == nullptr) {
        throwJava(env, "java/lang/NullPointerException", "positions must not be null");
        return;
    }
    if (type < 0 || type > 0xFFFF) {
        throwJava(env, "java/util/NoSuchElementException", "piece type id out of range");
        return;
    }
    const std::vector<float> positions = copyFloats(env, positionsXyzMeters);
    const std::vector<float> velocities = copyFloats(env, velocitiesXyzMetersPerSec);
    if (positions.size() % 3 != 0) {
        throwJava(env, "java/lang/IllegalArgumentException", "positions length must be a multiple of 3");
        return;
    }
    const auto count = static_cast<uint32_t>(positions.size() / 3);
    if (velocitiesXyzMetersPerSec != nullptr && velocities.size() != positions.size()) {
        throwJava(env, "java/lang/IllegalArgumentException", "velocities length must match positions");
        return;
    }
    if (outIndices != nullptr && static_cast<std::size_t>(env->GetArrayLength(outIndices)) < count) {
        throwJava(env, "java/lang/IllegalArgumentException", "outIndices is too small");
        return;
    }
    std::vector<uint32_t> indices(count);
    if (check(env, frcsim_pieces_spawn(world, static_cast<frcsim_piece_type_id>(type), positions.data(),
                                       velocitiesXyzMetersPerSec != nullptr ? velocities.data() : nullptr, count,
                                       indices.data()))) {
        return;
    }
    if (outIndices != nullptr && count > 0) {
        env->SetIntArrayRegion(outIndices, 0, static_cast<jsize>(count), reinterpret_cast<const jint*>(indices.data()));
    }
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceDespawn(JNIEnv* env, jclass, jlong handle, jint index) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_piece_despawn(world, static_cast<uint32_t>(index)));
    }
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceSetState(JNIEnv* env, jclass, jlong handle, jint index,
                                                                    jint state) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_piece_set_state(world, static_cast<uint32_t>(index), state));
    }
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceTeleport(
    JNIEnv* env, jclass, jlong handle, jint index, jfloat xMeters, jfloat yMeters, jfloat zMeters,
    jfloat vxMetersPerSec, jfloat vyMetersPerSec, jfloat vzMetersPerSec, jfloat wxRadPerSec, jfloat wyRadPerSec,
    jfloat wzRadPerSec) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float positionMeters[] = {xMeters, yMeters, zMeters};
        const float velocityMetersPerSec[] = {vxMetersPerSec, vyMetersPerSec, vzMetersPerSec};
        const float angularVelocityRadPerSec[] = {wxRadPerSec, wyRadPerSec, wzRadPerSec};
        check(env, frcsim_piece_teleport(world, static_cast<uint32_t>(index), positionMeters, velocityMetersPerSec,
                                         angularVelocityRadPerSec));
    }
}

JNIEXPORT jobject JNICALL Java_org_frcsim_jni_FrcSimJNI_piecePositionsBuffer(JNIEnv* env, jclass, jlong handle) {
    frcsim_world* world = worldFromHandle(env, handle);
    frcsim_piece_buffers buffers{};
    if (world == nullptr || check(env, frcsim_pieces_get_buffers(world, &buffers))) {
        return nullptr;
    }
    return env->NewDirectByteBuffer(const_cast<float*>(buffers.positions_xyz_meters),
                                    static_cast<jlong>(buffers.capacity) * 3 * sizeof(float));
}

JNIEXPORT jobject JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceStatesBuffer(JNIEnv* env, jclass, jlong handle) {
    frcsim_world* world = worldFromHandle(env, handle);
    frcsim_piece_buffers buffers{};
    if (world == nullptr || check(env, frcsim_pieces_get_buffers(world, &buffers))) {
        return nullptr;
    }
    return env->NewDirectByteBuffer(const_cast<uint8_t*>(buffers.states), static_cast<jlong>(buffers.capacity));
}

} // extern "C"
