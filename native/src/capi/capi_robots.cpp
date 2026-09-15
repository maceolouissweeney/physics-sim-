// C ABI: motor presets, swerve configuration, robots, and the shared robot I/O block.

#include <cstddef>
#include <stdexcept>

#include "capi/capi_internal.h"

static_assert(sizeof(frcsim_swerve_module_io) == 64);
static_assert(offsetof(frcsim_swerve_module_io, drive_rotor_position_radians) == 8);
static_assert(offsetof(frcsim_swerve_module_io, slip_speed_meters_per_sec) == 60);
static_assert(sizeof(frcsim_swerve_robot_io) == 608);
static_assert(offsetof(frcsim_swerve_robot_io, yaw_radians) == 24);
static_assert(offsetof(frcsim_swerve_robot_io, qx) == 32);
static_assert(offsetof(frcsim_swerve_robot_io, battery_volts) == 72);
static_assert(offsetof(frcsim_swerve_robot_io, module_count) == 84);
static_assert(offsetof(frcsim_swerve_robot_io, modules) == 88);
static_assert(offsetof(frcsim_swerve_robot_io, gyro_yaw_radians) == 600);
static_assert(frcsim::kMaxSwerveModules == FRCSIM_MAX_SWERVE_MODULES);
static_assert(static_cast<int>(frcsim::NeutralMode::Coast) == FRCSIM_NEUTRAL_COAST);

using namespace frcsim::capi;

void frcsim_world::applyRobotInputs() {
    for (std::size_t r = 0; r < robotIo.size(); ++r) {
        frcsim::SwerveRobot& robot = world.robots().swerve(static_cast<std::uint32_t>(r));
        const frcsim_swerve_robot_io& io = *robotIo[r];
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            robot.setModuleVoltages(m, io.modules[m].drive_command_volts, io.modules[m].steer_command_volts);
        }
    }
}

void frcsim_world::refreshRobotOutputs() {
    for (std::size_t r = 0; r < robotIo.size(); ++r) {
        const frcsim::SwerveRobot& robot = world.robots().swerve(static_cast<std::uint32_t>(r));
        frcsim_swerve_robot_io& io = *robotIo[r];
        const frcsim::RobotPose pose = robot.pose();
        io.x_meters = pose.positionMeters.GetX();
        io.y_meters = pose.positionMeters.GetY();
        io.z_meters = pose.positionMeters.GetZ();
        io.yaw_radians = robot.continuousYawRadians();
        io.gyro_yaw_radians = robot.measuredGyroYawRadians();
        io.qx = pose.rotation.GetX();
        io.qy = pose.rotation.GetY();
        io.qz = pose.rotation.GetZ();
        io.qw = pose.rotation.GetW();
        io.vx_meters_per_sec = pose.linearVelocityMetersPerSec.GetX();
        io.vy_meters_per_sec = pose.linearVelocityMetersPerSec.GetY();
        io.vz_meters_per_sec = pose.linearVelocityMetersPerSec.GetZ();
        io.wx_rad_per_sec = pose.angularVelocityRadPerSec.GetX();
        io.wy_rad_per_sec = pose.angularVelocityRadPerSec.GetY();
        io.wz_rad_per_sec = pose.angularVelocityRadPerSec.GetZ();
        io.battery_volts = robot.battery().voltageVolts();
        io.battery_current_amps = robot.battery().currentAmps();
        io.brownout = robot.battery().brownout() ? 1u : 0u;
        io.module_count = static_cast<std::uint32_t>(robot.moduleCount());
        for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
            const frcsim::SwerveModuleState& s = robot.module(m);
            frcsim_swerve_module_io& out = io.modules[m];
            out.drive_rotor_position_radians = robot.measuredDriveRotorPositionRadians(m);
            out.steer_angle_radians = s.steerAngleRadians;
            out.drive_rotor_velocity_rad_per_sec = static_cast<float>(robot.driveRotorVelocityRadPerSec(m));
            out.steer_velocity_rad_per_sec = s.steerVelocityRadPerSec;
            out.drive_applied_volts = s.driveAppliedVolts;
            out.drive_stator_current_amps = s.driveStatorCurrentAmps;
            out.drive_supply_current_amps = s.driveSupplyCurrentAmps;
            out.steer_applied_volts = s.steerAppliedVolts;
            out.steer_stator_current_amps = s.steerStatorCurrentAmps;
            out.steer_supply_current_amps = s.steerSupplyCurrentAmps;
            out.normal_force_newtons = s.normalForceNewtons;
            out.slip_speed_meters_per_sec = s.slipSpeedMetersPerSec;
        }
    }
}

namespace {

frcsim_motor_params toC(const frcsim::DcMotorParams& p) {
    return frcsim_motor_params{p.nominalVoltageVolts, p.stallTorqueNewtonMeters, p.stallCurrentAmps,
                               p.freeCurrentAmps,     p.freeSpeedRadPerSec,      p.count,
                               p.rotorInertiaKgMetersSq};
}

frcsim::DcMotorParams toCpp(const frcsim_motor_params& p) {
    frcsim::DcMotorParams m;
    m.nominalVoltageVolts = p.nominal_volts;
    m.stallTorqueNewtonMeters = p.stall_torque_newton_meters;
    m.stallCurrentAmps = p.stall_current_amps;
    m.freeCurrentAmps = p.free_current_amps;
    m.freeSpeedRadPerSec = p.free_speed_rad_per_sec;
    m.count = p.count;
    m.rotorInertiaKgMetersSq = p.rotor_inertia_kg_meters_sq;
    return m;
}

frcsim::NeutralMode toNeutralMode(int32_t mode) {
    if (mode != FRCSIM_NEUTRAL_BRAKE && mode != FRCSIM_NEUTRAL_COAST) {
        throw std::invalid_argument("neutral mode must be FRCSIM_NEUTRAL_BRAKE or FRCSIM_NEUTRAL_COAST");
    }
    return static_cast<frcsim::NeutralMode>(mode);
}

frcsim_swerve_module_config toC(const frcsim::SwerveModuleConfig& m) {
    frcsim_swerve_module_config c{};
    c.x_meters = m.xMeters;
    c.y_meters = m.yMeters;
    c.wheel_radius_meters = m.wheelRadiusMeters;
    c.wheel_width_meters = m.wheelWidthMeters;
    c.wheel_inertia_kg_meters_sq = m.wheelInertiaKgMetersSq;
    c.drive_motor = toC(m.driveMotor);
    c.drive_gear_ratio = m.driveGearRatio;
    c.drive_efficiency = m.driveEfficiency;
    c.drive_friction_torque_newton_meters = m.driveFrictionTorqueNewtonMeters;
    c.drive_stator_current_limit_amps = m.driveCurrentLimits.statorAmps;
    c.drive_supply_current_limit_amps = m.driveCurrentLimits.supplyAmps;
    c.drive_neutral_mode = static_cast<int32_t>(m.driveNeutralMode);
    c.steer_motor = toC(m.steerMotor);
    c.steer_gear_ratio = m.steerGearRatio;
    c.steer_efficiency = m.steerEfficiency;
    c.steer_inertia_kg_meters_sq = m.steerInertiaKgMetersSq;
    c.steer_friction_torque_newton_meters = m.steerFrictionTorqueNewtonMeters;
    c.steer_stator_current_limit_amps = m.steerCurrentLimits.statorAmps;
    c.steer_supply_current_limit_amps = m.steerCurrentLimits.supplyAmps;
    c.steer_neutral_mode = static_cast<int32_t>(m.steerNeutralMode);
    c.coupling_gear_ratio = m.couplingGearRatio;
    c.tire_static_friction = m.tire.staticFriction;
    c.tire_kinetic_friction = m.tire.kineticFriction;
    c.tire_transition_slip_speed_meters_per_sec = m.tire.transitionSlipSpeedMetersPerSec;
    c.scrub_radius_meters = m.scrubRadiusMeters;
    return c;
}

frcsim::SwerveModuleConfig toCpp(const frcsim_swerve_module_config& c) {
    frcsim::SwerveModuleConfig m;
    m.xMeters = c.x_meters;
    m.yMeters = c.y_meters;
    m.wheelRadiusMeters = c.wheel_radius_meters;
    m.wheelWidthMeters = c.wheel_width_meters;
    m.wheelInertiaKgMetersSq = c.wheel_inertia_kg_meters_sq;
    m.driveMotor = toCpp(c.drive_motor);
    m.driveGearRatio = c.drive_gear_ratio;
    m.driveEfficiency = c.drive_efficiency;
    m.driveFrictionTorqueNewtonMeters = c.drive_friction_torque_newton_meters;
    m.driveCurrentLimits = {c.drive_stator_current_limit_amps, c.drive_supply_current_limit_amps};
    m.driveNeutralMode = toNeutralMode(c.drive_neutral_mode);
    m.steerMotor = toCpp(c.steer_motor);
    m.steerGearRatio = c.steer_gear_ratio;
    m.steerEfficiency = c.steer_efficiency;
    m.steerInertiaKgMetersSq = c.steer_inertia_kg_meters_sq;
    m.steerFrictionTorqueNewtonMeters = c.steer_friction_torque_newton_meters;
    m.steerCurrentLimits = {c.steer_stator_current_limit_amps, c.steer_supply_current_limit_amps};
    m.steerNeutralMode = toNeutralMode(c.steer_neutral_mode);
    m.couplingGearRatio = c.coupling_gear_ratio;
    m.tire = {c.tire_static_friction, c.tire_kinetic_friction, c.tire_transition_slip_speed_meters_per_sec};
    m.scrubRadiusMeters = c.scrub_radius_meters;
    return m;
}

} // namespace

extern "C" {

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
        case FRCSIM_MOTOR_MINION: p = frcsim::DcMotorParams::minion(count); break;
        default: return fail(FRCSIM_ERR_INVALID_ARGUMENT, "unknown motor preset");
        }
        *out_params = toC(p);
        return FRCSIM_OK;
    });
}

void frcsim_swerve_config_init(frcsim_swerve_config* config, float track_width_meters, float wheel_base_meters) {
    if (config == nullptr) {
        return;
    }
    const frcsim::SwerveDriveConfig d = frcsim::makeRectangularSwerve(track_width_meters, wheel_base_meters);
    *config = frcsim_swerve_config{};
    config->struct_size = sizeof(frcsim_swerve_config);
    config->mass_kg = d.massKg;
    config->frame_half_x_meters = d.frameHalfXMeters;
    config->frame_half_y_meters = d.frameHalfYMeters;
    config->bumper_bottom_meters = d.bumperBottomMeters;
    config->bumper_height_meters = d.bumperHeightMeters;
    config->com_x_meters = d.comXMeters;
    config->com_y_meters = d.comYMeters;
    config->com_height_meters = d.comHeightMeters;
    config->yaw_inertia_kg_meters_sq = d.yawInertiaKgMetersSq;
    config->bumper_material = FRCSIM_MATERIAL_DEFAULT;
    config->suspension_travel_meters = d.suspension.travelMeters;
    config->suspension_frequency_hz = d.suspension.frequencyHz;
    config->suspension_damping_ratio = d.suspension.dampingRatio;
    config->battery_open_circuit_volts = d.battery.openCircuitVolts;
    config->battery_internal_resistance_ohms = d.battery.internalResistanceOhms;
    config->battery_brownout_volts = d.battery.brownoutVolts;
    config->battery_brownout_recovery_volts = d.battery.brownoutRecoveryVolts;
    config->module_count = static_cast<uint32_t>(d.modules.size());
    config->gyro_yaw_noise_radians = d.sensors.gyroYawNoiseRadians;
    config->gyro_yaw_drift_rate_rad_per_sec = d.sensors.gyroYawDriftRateRadPerSec;
    config->gyro_scale_error = d.sensors.gyroScaleError;
    config->drive_encoder_counts_per_rev = d.sensors.driveEncoderCountsPerRev;
    config->sensor_seed = d.sensors.seed;
    for (std::size_t i = 0; i < d.modules.size(); ++i) {
        config->modules[i] = toC(d.modules[i]);
    }
}

frcsim_status frcsim_robot_add_swerve(frcsim_world* world, const frcsim_swerve_config* config, float x_meters,
                                     float y_meters, float yaw_radians, uint32_t* out_robot) {
    return withWorld(world, [&](frcsim_world& w) {
        requireNonNull(config, "config");
        if (config->struct_size != sizeof(frcsim_swerve_config)) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT,
                        "frcsim_swerve_config.struct_size mismatch; call frcsim_swerve_config_init()");
        }
        if (config->module_count > FRCSIM_MAX_SWERVE_MODULES) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "module_count exceeds FRCSIM_MAX_SWERVE_MODULES");
        }

        frcsim::SwerveDriveConfig d;
        d.massKg = config->mass_kg;
        d.frameHalfXMeters = config->frame_half_x_meters;
        d.frameHalfYMeters = config->frame_half_y_meters;
        d.bumperBottomMeters = config->bumper_bottom_meters;
        d.bumperHeightMeters = config->bumper_height_meters;
        d.comXMeters = config->com_x_meters;
        d.comYMeters = config->com_y_meters;
        d.comHeightMeters = config->com_height_meters;
        d.yawInertiaKgMetersSq = config->yaw_inertia_kg_meters_sq;
        d.bumperMaterial = config->bumper_material;
        d.suspension = {config->suspension_travel_meters, config->suspension_frequency_hz,
                        config->suspension_damping_ratio};
        d.battery = {config->battery_open_circuit_volts, config->battery_internal_resistance_ohms,
                     config->battery_brownout_volts, config->battery_brownout_recovery_volts};
        d.sensors = {config->gyro_yaw_noise_radians, config->gyro_yaw_drift_rate_rad_per_sec, config->gyro_scale_error,
                     config->drive_encoder_counts_per_rev, config->sensor_seed};
        d.modules.reserve(config->module_count);
        for (uint32_t i = 0; i < config->module_count; ++i) {
            d.modules.push_back(toCpp(config->modules[i]));
        }

        w.robotIo.reserve(w.robotIo.size() + 1); // reserve first so push_back below cannot throw after adding
        auto io = std::make_unique<frcsim_swerve_robot_io>();
        const std::uint32_t index = w.world.robots().addSwerve(d, x_meters, y_meters, yaw_radians);
        w.robotIo.push_back(std::move(io));
        w.refreshRobotOutputs();
        writeIndex(out_robot, index);
        return FRCSIM_OK;
    });
}

frcsim_status frcsim_robot_reset_pose(frcsim_world* world, uint32_t robot, float x_meters, float y_meters,
                                     float yaw_radians) {
    return withWorld(world, [&](frcsim_world& w) {
        w.world.robots().swerve(robot).resetPose(x_meters, y_meters, yaw_radians);
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

} // extern "C"
