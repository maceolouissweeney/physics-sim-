// JNI bindings for org.frcsim.jni.FrcSimJNI: swerve robots and the shared robot I/O block.

#include <algorithm>
#include <cmath>

#include "jni/jni_support.h"

using namespace frcsim::jni;

namespace {

// Packing order of SwerveDriveConfig.packRobot() / SwerveModuleConfig.packInto() in Java. Keep in sync.
enum RobotParam : int {
    kMassKg, kFrameHalfXMeters, kFrameHalfYMeters, kBumperBottomMeters, kBumperHeightMeters, kComXMeters,
    kComYMeters, kComHeightMeters, kYawInertiaKgMetersSq,
    kSuspensionTravelMeters, kSuspensionFrequencyHz, kSuspensionDampingRatio,
    kBatteryOpenCircuitVolts, kBatteryInternalResistanceOhms, kBatteryBrownoutVolts, kBatteryBrownoutRecoveryVolts,
    kGyroYawNoiseRadians, kGyroYawDriftRateRadPerSec, kGyroScaleError, kDriveEncoderCountsPerRev, kSensorSeed,
    kRobotParamCount
};
enum ModuleParam : int {
    kModuleXMeters = 0, kModuleYMeters = 1, kWheelRadiusMeters = 2, kWheelWidthMeters = 3,
    kWheelInertiaKgMetersSq = 4,
    kDriveMotor = 5, // 7 motor fields
    kDriveGearRatio = 12, kDriveEfficiency = 13, kDriveFrictionTorqueNewtonMeters = 14,
    kDriveStatorLimitAmps = 15, kDriveSupplyLimitAmps = 16, kDriveNeutralMode = 17,
    kSteerMotor = 18, // 7 motor fields
    kSteerGearRatio = 25, kSteerEfficiency = 26, kSteerInertiaKgMetersSq = 27, kSteerFrictionTorqueNewtonMeters = 28,
    kSteerStatorLimitAmps = 29, kSteerSupplyLimitAmps = 30, kSteerNeutralMode = 31,
    kCouplingGearRatio = 32,
    kTireStaticFriction = 33, kTireKineticFriction = 34, kTireTransitionSlipSpeedMetersPerSec = 35,
    kScrubRadiusMeters = 36,
    kModuleParamCount = 37
};

frcsim_motor_params unpackMotor(const float* p) {
    return frcsim_motor_params{p[0], p[1], p[2], p[3], p[4], static_cast<int32_t>(std::lround(p[5])), p[6]};
}

uint32_t toUint(float value) {
    return static_cast<uint32_t>(std::lround(std::max(0.0f, value)));
}

} // namespace

extern "C" {

JNIEXPORT jint JNICALL Java_org_frcsim_jni_FrcSimJNI_robotAddSwerve(JNIEnv* env, jclass, jlong handle,
                                                                     jfloatArray robotParams, jint bumperMaterial,
                                                                     jfloatArray moduleParams, jfloat xMeters,
                                                                     jfloat yMeters, jfloat yawRadians) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return -1;
    }
    if (robotParams == nullptr || moduleParams == nullptr) {
        throwJava(env, "java/lang/NullPointerException", "robot and module parameters must not be null");
        return -1;
    }
    const std::vector<float> r = copyFloats(env, robotParams);
    const std::vector<float> m = copyFloats(env, moduleParams);
    const std::size_t moduleCount = m.size() / kModuleParamCount;
    if (r.size() != kRobotParamCount || m.size() % kModuleParamCount != 0 || moduleCount == 0 ||
        moduleCount > FRCSIM_MAX_SWERVE_MODULES) {
        throwJava(env, "java/lang/IllegalArgumentException", "malformed packed swerve configuration");
        return -1;
    }
    if (bumperMaterial < 0 || bumperMaterial > 255) {
        throwJava(env, "java/util/NoSuchElementException", "bumper material id out of range");
        return -1;
    }

    frcsim_swerve_config c;
    frcsim_swerve_config_init(&c, 0.5f, 0.5f);
    c.mass_kg = r[kMassKg];
    c.frame_half_x_meters = r[kFrameHalfXMeters];
    c.frame_half_y_meters = r[kFrameHalfYMeters];
    c.bumper_bottom_meters = r[kBumperBottomMeters];
    c.bumper_height_meters = r[kBumperHeightMeters];
    c.com_x_meters = r[kComXMeters];
    c.com_y_meters = r[kComYMeters];
    c.com_height_meters = r[kComHeightMeters];
    c.yaw_inertia_kg_meters_sq = r[kYawInertiaKgMetersSq];
    c.bumper_material = static_cast<frcsim_material_id>(bumperMaterial);
    c.suspension_travel_meters = r[kSuspensionTravelMeters];
    c.suspension_frequency_hz = r[kSuspensionFrequencyHz];
    c.suspension_damping_ratio = r[kSuspensionDampingRatio];
    c.battery_open_circuit_volts = r[kBatteryOpenCircuitVolts];
    c.battery_internal_resistance_ohms = r[kBatteryInternalResistanceOhms];
    c.battery_brownout_volts = r[kBatteryBrownoutVolts];
    c.battery_brownout_recovery_volts = r[kBatteryBrownoutRecoveryVolts];
    c.gyro_yaw_noise_radians = r[kGyroYawNoiseRadians];
    c.gyro_yaw_drift_rate_rad_per_sec = r[kGyroYawDriftRateRadPerSec];
    c.gyro_scale_error = r[kGyroScaleError];
    c.drive_encoder_counts_per_rev = toUint(r[kDriveEncoderCountsPerRev]);
    c.sensor_seed = toUint(r[kSensorSeed]);
    c.module_count = static_cast<uint32_t>(moduleCount);
    for (std::size_t i = 0; i < moduleCount; ++i) {
        const float* p = m.data() + i * kModuleParamCount;
        frcsim_swerve_module_config& mc = c.modules[i];
        mc.x_meters = p[kModuleXMeters];
        mc.y_meters = p[kModuleYMeters];
        mc.wheel_radius_meters = p[kWheelRadiusMeters];
        mc.wheel_width_meters = p[kWheelWidthMeters];
        mc.wheel_inertia_kg_meters_sq = p[kWheelInertiaKgMetersSq];
        mc.drive_motor = unpackMotor(p + kDriveMotor);
        mc.drive_gear_ratio = p[kDriveGearRatio];
        mc.drive_efficiency = p[kDriveEfficiency];
        mc.drive_friction_torque_newton_meters = p[kDriveFrictionTorqueNewtonMeters];
        mc.drive_stator_current_limit_amps = p[kDriveStatorLimitAmps];
        mc.drive_supply_current_limit_amps = p[kDriveSupplyLimitAmps];
        mc.drive_neutral_mode = static_cast<int32_t>(std::lround(p[kDriveNeutralMode]));
        mc.steer_motor = unpackMotor(p + kSteerMotor);
        mc.steer_gear_ratio = p[kSteerGearRatio];
        mc.steer_efficiency = p[kSteerEfficiency];
        mc.steer_inertia_kg_meters_sq = p[kSteerInertiaKgMetersSq];
        mc.steer_friction_torque_newton_meters = p[kSteerFrictionTorqueNewtonMeters];
        mc.steer_stator_current_limit_amps = p[kSteerStatorLimitAmps];
        mc.steer_supply_current_limit_amps = p[kSteerSupplyLimitAmps];
        mc.steer_neutral_mode = static_cast<int32_t>(std::lround(p[kSteerNeutralMode]));
        mc.coupling_gear_ratio = p[kCouplingGearRatio];
        mc.tire_static_friction = p[kTireStaticFriction];
        mc.tire_kinetic_friction = p[kTireKineticFriction];
        mc.tire_transition_slip_speed_meters_per_sec = p[kTireTransitionSlipSpeedMetersPerSec];
        mc.scrub_radius_meters = p[kScrubRadiusMeters];
    }

    uint32_t robot = 0;
    return toJavaIndex(env, frcsim_robot_add_swerve(world, &c, xMeters, yMeters, yawRadians, &robot), robot);
}

JNIEXPORT jobject JNICALL Java_org_frcsim_jni_FrcSimJNI_robotIoBuffer(JNIEnv* env, jclass, jlong handle, jint robot) {
    frcsim_world* world = worldFromHandle(env, handle);
    if (world == nullptr) {
        return nullptr;
    }
    frcsim_swerve_robot_io* io = robot >= 0 ? frcsim_robot_io(world, static_cast<uint32_t>(robot)) : nullptr;
    if (io == nullptr) {
        throwJava(env, "java/util/NoSuchElementException", "robot index out of range");
        return nullptr;
    }
    return env->NewDirectByteBuffer(io, static_cast<jlong>(sizeof(frcsim_swerve_robot_io)));
}

JNIEXPORT void JNICALL Java_org_frcsim_jni_FrcSimJNI_robotResetPose(JNIEnv* env, jclass, jlong handle, jint robot,
                                                                     jfloat xMeters, jfloat yMeters,
                                                                     jfloat yawRadians) {
    if (frcsim_world* world = worldFromHandle(env, handle)) {
        check(env, frcsim_robot_reset_pose(world, static_cast<uint32_t>(robot), xMeters, yMeters, yawRadians));
    }
}

/// Offsets Java verifies against its constants at load time (SwerveRobot.Layout). Order is part of the contract.
JNIEXPORT jintArray JNICALL Java_org_frcsim_jni_FrcSimJNI_robotIoLayout(JNIEnv* env, jclass) {
    const jint layout[] = {
        static_cast<jint>(sizeof(frcsim_swerve_robot_io)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, yaw_radians)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, qx)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, vx_meters_per_sec)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, wx_rad_per_sec)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, battery_volts)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, brownout)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, module_count)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, modules)),
        static_cast<jint>(sizeof(frcsim_swerve_module_io)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, drive_rotor_position_radians)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, steer_angle_radians)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, drive_rotor_velocity_rad_per_sec)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, steer_velocity_rad_per_sec)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, drive_applied_volts)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, normal_force_newtons)),
        static_cast<jint>(offsetof(frcsim_swerve_module_io, slip_speed_meters_per_sec)),
        static_cast<jint>(offsetof(frcsim_swerve_robot_io, gyro_yaw_radians)),
    };
    return toJavaIntArray(env, layout, sizeof(layout) / sizeof(layout[0]));
}

} // extern "C"
