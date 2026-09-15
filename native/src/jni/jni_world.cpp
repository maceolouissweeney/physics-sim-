// JNI bindings for org.frcsim.jni.FrcSimJNI: library, world, materials, field, kinematic bodies.

#include <string>

#include "jni/jni_support.h"

using namespace frcsim::jni;

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
    jint tempAllocatorBytes, jdouble gravityZMetersPerSecSq, jint maxPieces,
    jfloat minVelocityForRestitutionMetersPerSec, jfloat timeBeforeSleepSeconds,
    jfloat sleepVelocityThresholdMetersPerSec, jint solverVelocitySteps, jint solverPositionSteps) {
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
    config.gravity_z_meters_per_sec_sq = gravityZMetersPerSecSq;
    config.max_pieces = static_cast<uint32_t>(maxPieces);
    config.min_velocity_for_restitution_meters_per_sec = minVelocityForRestitutionMetersPerSec;
    config.time_before_sleep_seconds = timeBeforeSleepSeconds;
    config.sleep_velocity_threshold_meters_per_sec = sleepVelocityThresholdMetersPerSec;
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

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_worldStep(JNIEnv* env, jclass, jlong handle, jdouble dtSeconds,
                                                                jint substeps) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_world_step(world, dtSeconds, substeps));
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
    return toJavaIntArray(env, layout, sizeof(layout) / sizeof(layout[0]));
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

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldAddGround(JNIEnv* env, jclass, jlong handle,
                                                                     jfloat heightMeters, jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return -1;
    }
    uint32_t index = 0;
    return toJavaIndex(env,
                       frcsim_field_add_ground(world, heightMeters, static_cast<frcsim_material_id>(material), &index),
                       index);
}

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldAddBox(
    JNIEnv* env, jclass, jlong handle, jstring name, jfloat centerXMeters, jfloat centerYMeters, jfloat centerZMeters,
    jfloat halfXMeters, jfloat halfYMeters, jfloat halfZMeters, jfloat qx, jfloat qy, jfloat qz, jfloat qw,
    jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    JavaString n(env, name);
    if (world == nullptr || n.get() == nullptr) {
        return -1;
    }
    const float centerMeters[] = {centerXMeters, centerYMeters, centerZMeters};
    const float halfExtentsMeters[] = {halfXMeters, halfYMeters, halfZMeters};
    const float rotation[] = {qx, qy, qz, qw};
    uint32_t index = 0;
    return toJavaIndex(env,
                       frcsim_field_add_box(world, n.get(), centerMeters, halfExtentsMeters, rotation,
                                            static_cast<frcsim_material_id>(material), &index),
                       index);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_fieldSetBounds(JNIEnv* env, jclass, jlong handle,
                                                                     jfloat minXMeters, jfloat minYMeters,
                                                                     jfloat minZMeters, jfloat maxXMeters,
                                                                     jfloat maxYMeters, jfloat maxZMeters) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float minMeters[] = {minXMeters, minYMeters, minZMeters};
        const float maxMeters[] = {maxXMeters, maxYMeters, maxZMeters};
        check(env, frcsim_field_set_bounds(world, minMeters, maxMeters));
    }
}

/* ---- Kinematic bodies ---------------------------------------------------------------------------- */

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_kinematicAddBox(
    JNIEnv* env, jclass, jlong handle, jfloat centerXMeters, jfloat centerYMeters, jfloat centerZMeters,
    jfloat halfXMeters, jfloat halfYMeters, jfloat halfZMeters, jfloat qx, jfloat qy, jfloat qz, jfloat qw,
    jint material) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return -1;
    }
    const float centerMeters[] = {centerXMeters, centerYMeters, centerZMeters};
    const float halfExtentsMeters[] = {halfXMeters, halfYMeters, halfZMeters};
    const float rotation[] = {qx, qy, qz, qw};
    uint32_t index = 0;
    return toJavaIndex(env,
                       frcsim_kinematic_add_box(world, centerMeters, halfExtentsMeters, rotation,
                                                static_cast<frcsim_material_id>(material), &index),
                       index);
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_kinematicMoveTo(JNIEnv* env, jclass, jlong handle, jint index,
                                                                      jfloat xMeters, jfloat yMeters, jfloat zMeters,
                                                                      jfloat qx, jfloat qy, jfloat qz, jfloat qw,
                                                                      jdouble dtSeconds) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        const float positionMeters[] = {xMeters, yMeters, zMeters};
        const float rotation[] = {qx, qy, qz, qw};
        check(env, frcsim_kinematic_move_to(world, static_cast<uint32_t>(index), positionMeters, rotation, dtSeconds));
    }
}

} // extern "C"
