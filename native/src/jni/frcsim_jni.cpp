// JNI bindings for org.frcsim.jni.FrcSimJNI.
//
// Rules (see CLAUDE.md §8): this file calls only the C ABI, never C++ internals, so an FFM binding can
// reuse the same surface. Failures become Java exceptions; nothing here may crash the JVM on bad input.
// Arrays are copied (Get*ArrayRegion) rather than pinned: these calls happen at setup time, not per tick.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <jni.h>

#include <frcsim/frcsim_c.h>

namespace {

void throwJava(JNIEnv* env, const char* className, const char* message) {
    jclass cls = env->FindClass(className);
    if (cls != nullptr) {
        env->ThrowNew(cls, message);
        env->DeleteLocalRef(cls);
    }
}

/// Throws the Java exception matching a failed status. Returns true if an exception was thrown.
bool check(JNIEnv* env, frcsim_status status) {
    if (status == FRCSIM_OK) {
        return false;
    }
    const char* message = frcsim_last_error();
    if (message == nullptr || message[0] == '\0') {
        message = frcsim_status_string(status);
    }
    switch (status) {
    case FRCSIM_ERR_INVALID_ARGUMENT:
        throwJava(env, "java/lang/IllegalArgumentException", message);
        break;
    case FRCSIM_ERR_OUT_OF_MEMORY:
        throwJava(env, "java/lang/OutOfMemoryError", message);
        break;
    case FRCSIM_ERR_CAPACITY_EXCEEDED:
        throwJava(env, "org/frcsim/CapacityExceededException", message);
        break;
    case FRCSIM_ERR_NOT_FOUND:
        throwJava(env, "java/util/NoSuchElementException", message);
        break;
    default:
        throwJava(env, "org/frcsim/FrcSimException", message);
        break;
    }
    return true;
}

frcsim_world* worldFromHandle(JNIEnv* env, jlong handle) {
    if (handle == 0) {
        throwJava(env, "java/lang/IllegalStateException", "world has been closed");
        return nullptr;
    }
    return reinterpret_cast<frcsim_world*>(static_cast<intptr_t>(handle));
}

/// Modified-UTF-8 view of a Java string (fine for identifiers; use byte[] for arbitrary text).
class JavaString {
public:
    JavaString(JNIEnv* env, jstring string)
        : m_env(env), m_string(string), m_chars(string != nullptr ? env->GetStringUTFChars(string, nullptr) : nullptr) {
        if (string == nullptr) {
            throwJava(env, "java/lang/NullPointerException", "string argument must not be null");
        }
    }
    ~JavaString() {
        if (m_chars != nullptr) {
            m_env->ReleaseStringUTFChars(m_string, m_chars);
        }
    }
    JavaString(const JavaString&) = delete;
    JavaString& operator=(const JavaString&) = delete;

    [[nodiscard]] const char* get() const { return m_chars; }

private:
    JNIEnv* m_env;
    jstring m_string;
    const char* m_chars;
};

std::vector<float> copyFloats(JNIEnv* env, jfloatArray array) {
    std::vector<float> values;
    if (array != nullptr) {
        values.resize(static_cast<std::size_t>(env->GetArrayLength(array)));
        env->GetFloatArrayRegion(array, 0, static_cast<jsize>(values.size()), values.data());
    }
    return values;
}

} // namespace

extern "C" {

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM*, void*) {
    return JNI_VERSION_1_8;
}

/* ---- Library ------------------------------------------------------------------------------------- */

JNIEXPORT jstring JNICALL Java_org_frcsim_jni_FrcSimJNI_versionString(JNIEnv* env, jclass) {
    return env->NewStringUTF(frcsim_version_string());
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_abiVersion(JNIEnv*, jclass) {
    return static_cast<jint>(frcsim_abi_version());
}

/* ---- World --------------------------------------------------------------------------------------- */

JNIEXPORT jlong JNICALL Java_org_frcsim_jni_FrcSimJNI_worldCreate(
    JNIEnv* env, jclass, jint maxBodies, jint maxBodyPairs, jint maxContactConstraints, jint workerThreads,
    jint tempAllocatorBytes, jdouble gravityZ, jint maxPieces, jfloat minVelocityForRestitution,
    jfloat timeBeforeSleep, jfloat sleepVelocityThreshold, jint solverVelocitySteps, jint solverPositionSteps) {
    if (maxBodies <= 0 || maxBodyPairs <= 0 || maxContactConstraints <= 0 || tempAllocatorBytes <= 0 ||
        maxPieces <= 0 || solverVelocitySteps < 0 || solverPositionSteps < 0) {
        throwJava(env, "java/lang/IllegalArgumentException", "capacities and solver steps must be positive");
        return 0;
    }
    frcsim_world_config config;
    frcsim_world_config_init(&config);
    config.max_bodies = static_cast<uint32_t>(maxBodies);
    config.max_body_pairs = static_cast<uint32_t>(maxBodyPairs);
    config.max_contact_constraints = static_cast<uint32_t>(maxContactConstraints);
    config.worker_threads = workerThreads;
    config.temp_allocator_bytes = static_cast<uint32_t>(tempAllocatorBytes);
    config.gravity_z = gravityZ;
    config.max_pieces = static_cast<uint32_t>(maxPieces);
    config.min_velocity_for_restitution = minVelocityForRestitution;
    config.time_before_sleep = timeBeforeSleep;
    config.sleep_velocity_threshold = sleepVelocityThreshold;
    config.solver_velocity_steps = static_cast<uint32_t>(solverVelocitySteps);
    config.solver_position_steps = static_cast<uint32_t>(solverPositionSteps);

    frcsim_world* world = nullptr;
    if (check(env, frcsim_world_create(&config, &world))) {
        return 0;
    }
    return static_cast<jlong>(reinterpret_cast<intptr_t>(world));
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_worldDestroy(JNIEnv*, jclass, jlong handle) {
    frcsim_world_destroy(reinterpret_cast<frcsim_world*>(static_cast<intptr_t>(handle)));
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_worldStep(JNIEnv* env, jclass, jlong handle, jdouble dt,
                                                                jint substeps) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_world_step(world, dt, substeps));
    }
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_worldOptimizeBroadPhase(JNIEnv* env, jclass, jlong handle) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_world_optimize_broadphase(world));
    }
}

JNIEXPORT jobject JNICALL Java_org_frcsim_jni_FrcSimJNI_worldStatsBuffer(JNIEnv* env, jclass, jlong handle) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return nullptr;
    }
    // The block is read-only for Java; NewDirectByteBuffer requires a non-const pointer.
    auto* stats = const_cast<frcsim_world_stats*>(frcsim_world_stats_ptr(world));
    return env->NewDirectByteBuffer(stats, static_cast<jlong>(sizeof(frcsim_world_stats)));
}

JNIEXPORT jintArray JNICALL Java_org_frcsim_jni_FrcSimJNI_worldStatsLayout(JNIEnv* env, jclass) {
    const jint layout[] = {
        static_cast<jint>(sizeof(frcsim_world_stats)),
        static_cast<jint>(offsetof(frcsim_world_stats, time_seconds)),
        static_cast<jint>(offsetof(frcsim_world_stats, last_step_wall_seconds)),
        static_cast<jint>(offsetof(frcsim_world_stats, substep_count)),
        static_cast<jint>(offsetof(frcsim_world_stats, active_bodies)),
        static_cast<jint>(offsetof(frcsim_world_stats, update_error_flags)),
        static_cast<jint>(offsetof(frcsim_world_stats, piece_high_water)),
        static_cast<jint>(offsetof(frcsim_world_stats, pieces_simulated)),
    };
    const jsize n = static_cast<jsize>(sizeof(layout) / sizeof(layout[0]));
    jintArray result = env->NewIntArray(n);
    if (result != nullptr) {
        env->SetIntArrayRegion(result, 0, n, layout);
    }
    return result;
}

/* ---- Materials ----------------------------------------------------------------------------------- */

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_materialAdd(JNIEnv* env, jclass, jlong handle, jstring name,
                                                                  jfloat friction, jfloat restitution) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    frcsim_material_id id = 0;
    return check(env, frcsim_material_add(world, n.get(), friction, restitution, &id)) ? -1 : id;
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_materialFind(JNIEnv* env, jclass, jlong handle, jstring name) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    frcsim_material_id id = 0;
    const frcsim_status status = frcsim_material_find(world, n.get(), &id);
    if (status == FRCSIM_ERR_NOT_FOUND) {
        return -1; // absence is a normal answer for find
    }
    return check(env, status) ? -1 : id;
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_materialSetPair(JNIEnv* env, jclass, jlong handle, jint a,
                                                                      jint b, jfloat friction, jfloat restitution) {
    if (a < 0 || a > 255 || b < 0 || b > 255) {
        throwJava(env, "java/util/NoSuchElementException", "material id out of range");
        return;
    }
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_material_set_pair(world, static_cast<frcsim_material_id>(a),
                                            static_cast<frcsim_material_id>(b), friction, restitution));
    }
}

/* ---- Field --------------------------------------------------------------------------------------- */

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldLoadJson(JNIEnv* env, jclass, jlong handle,
                                                                    jbyteArray jsonUtf8) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return;
    }
    if (jsonUtf8 == nullptr) {
        throwJava(env, "java/lang/NullPointerException", "json must not be null");
        return;
    }
    std::string json(static_cast<std::size_t>(env->GetArrayLength(jsonUtf8)), '\0');
    env->GetByteArrayRegion(jsonUtf8, 0, static_cast<jsize>(json.size()), reinterpret_cast<jbyte*>(json.data()));
    check(env, frcsim_field_load_json(world, json.data(), json.size()));
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldAddGround(JNIEnv* env, jclass, jlong handle, jfloat height,
                                                                     jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return -1;
    }
    uint32_t index = 0;
    return check(env, frcsim_field_add_ground(world, height, static_cast<frcsim_material_id>(material), &index))
               ? -1
               : static_cast<jint>(index);
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldAddBox(JNIEnv* env, jclass, jlong handle, jstring name,
                                                                  jfloat cx, jfloat cy, jfloat cz, jfloat hx,
                                                                  jfloat hy, jfloat hz, jfloat qx, jfloat qy,
                                                                  jfloat qz, jfloat qw, jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    const float center[] = {cx, cy, cz};
    const float half[] = {hx, hy, hz};
    const float rotation[] = {qx, qy, qz, qw};
    uint32_t index = 0;
    return check(env, frcsim_field_add_box(world, n.get(), center, half, rotation,
                                           static_cast<frcsim_material_id>(material), &index))
               ? -1
               : static_cast<jint>(index);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldSetBounds(JNIEnv* env, jclass, jlong handle, jfloat minX,
                                                                     jfloat minY, jfloat minZ, jfloat maxX,
                                                                     jfloat maxY, jfloat maxZ) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float min[] = {minX, minY, minZ};
        const float max[] = {maxX, maxY, maxZ};
        check(env, frcsim_field_set_bounds(world, min, max));
    }
}

/* ---- Game pieces --------------------------------------------------------------------------------- */

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceTypeAdd(JNIEnv* env, jclass, jlong handle, jstring name,
                                                                   jint shape, jfloat radius, jfloat halfHeight,
                                                                   jfloat hx, jfloat hy, jfloat hz, jfloat mass,
                                                                   jint material, jfloat maxAngularVelocity) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    frcsim_piece_type_desc desc;
    frcsim_piece_type_desc_init(&desc);
    desc.name = n.get();
    desc.shape = shape;
    desc.radius = radius;
    desc.half_height = halfHeight;
    desc.half_extents[0] = hx;
    desc.half_extents[1] = hy;
    desc.half_extents[2] = hz;
    desc.mass = mass;
    desc.material = static_cast<frcsim_material_id>(material);
    desc.max_angular_velocity = maxAngularVelocity;
    frcsim_piece_type_id id = 0;
    return check(env, frcsim_piece_type_add(world, &desc, &id)) ? -1 : static_cast<jint>(id);
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
    return check(env, status) ? -1 : static_cast<jint>(id);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_piecesSpawn(JNIEnv* env, jclass, jlong handle, jint type,
                                                                  jfloatArray positionsXyz, jfloatArray velocitiesXyz,
                                                                  jintArray outIndices) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return;
    }
    if (positionsXyz == nullptr) {
        throwJava(env, "java/lang/NullPointerException", "positions must not be null");
        return;
    }
    if (type < 0 || type > 0xFFFF) {
        throwJava(env, "java/util/NoSuchElementException", "piece type id out of range");
        return;
    }
    const std::vector<float> positions = copyFloats(env, positionsXyz);
    const std::vector<float> velocities = copyFloats(env, velocitiesXyz);
    if (positions.size() % 3 != 0) {
        throwJava(env, "java/lang/IllegalArgumentException", "positions length must be a multiple of 3");
        return;
    }
    const auto count = static_cast<uint32_t>(positions.size() / 3);
    if (velocitiesXyz != nullptr && velocities.size() != positions.size()) {
        throwJava(env, "java/lang/IllegalArgumentException", "velocities length must match positions");
        return;
    }
    if (outIndices != nullptr && static_cast<std::size_t>(env->GetArrayLength(outIndices)) < count) {
        throwJava(env, "java/lang/IllegalArgumentException", "outIndices is too small");
        return;
    }
    std::vector<uint32_t> indices(count);
    if (check(env, frcsim_pieces_spawn(world, static_cast<frcsim_piece_type_id>(type), positions.data(),
                                       velocitiesXyz != nullptr ? velocities.data() : nullptr, count,
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

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_pieceTeleport(JNIEnv* env, jclass, jlong handle, jint index,
                                                                    jfloat x, jfloat y, jfloat z, jfloat vx, jfloat vy,
                                                                    jfloat vz, jfloat wx, jfloat wy, jfloat wz) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float p[] = {x, y, z};
        const float v[] = {vx, vy, vz};
        const float w[] = {wx, wy, wz};
        check(env, frcsim_piece_teleport(world, static_cast<uint32_t>(index), p, v, w));
    }
}

JNIEXPORT jobject JNICALL Java_org_frcsim_jni_FrcSimJNI_piecePositionsBuffer(JNIEnv* env, jclass, jlong handle) {
    frcsim_world* world = worldFromHandle(env, handle);
    frcsim_piece_buffers buffers{};
    if (world == nullptr || check(env, frcsim_pieces_get_buffers(world, &buffers))) {
        return nullptr;
    }
    return env->NewDirectByteBuffer(const_cast<float*>(buffers.positions_xyz),
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

/* ---- Kinematic bodies ---------------------------------------------------------------------------- */

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_kinematicAddBox(JNIEnv* env, jclass, jlong handle, jfloat cx,
                                                                      jfloat cy, jfloat cz, jfloat hx, jfloat hy,
                                                                      jfloat hz, jfloat qx, jfloat qy, jfloat qz,
                                                                      jfloat qw, jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return -1;
    }
    const float center[] = {cx, cy, cz};
    const float half[] = {hx, hy, hz};
    const float rotation[] = {qx, qy, qz, qw};
    uint32_t index = 0;
    return check(env, frcsim_kinematic_add_box(world, center, half, rotation,
                                               static_cast<frcsim_material_id>(material), &index))
               ? -1
               : static_cast<jint>(index);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_kinematicMoveTo(JNIEnv* env, jclass, jlong handle, jint index,
                                                                      jfloat x, jfloat y, jfloat z, jfloat qx,
                                                                      jfloat qy, jfloat qz, jfloat qw, jdouble dt) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float position[] = {x, y, z};
        const float rotation[] = {qx, qy, qz, qw};
        check(env, frcsim_kinematic_move_to(world, static_cast<uint32_t>(index), position, rotation, dt));
    }
}

} // extern "C"
