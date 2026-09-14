#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "drive/battery.h"
#include "drive/dc_motor.h"
#include "drive/tire_model.h"
#include "world/materials.h"

namespace frcsim {

/// One swerve module. Defaults: SDS MK4i L2 with Kraken X60 drive and steer (docs/models/swerve.md).
struct SwerveModuleConfig {
    float x = 0.0f; ///< module center in the robot frame (m), +X forward
    float y = 0.0f; ///< +Y left

    float wheelRadius = 0.0508f;  ///< m (4 in wheel)
    float wheelWidth = 0.038f;    ///< m
    float wheelInertia = 3.0e-4f; ///< kg·m² about the axle, excluding the motor rotor. CALIBRATE

    DcMotorParams driveMotor = DcMotorParams::krakenX60();
    float driveGearRatio = 6.75f;      ///< motor rotations per wheel rotation
    float driveEfficiency = 0.95f;
    float driveFrictionTorque = 0.2f;  ///< N·m at the wheel. CALIBRATE
    CurrentLimits driveCurrentLimits{80.0f, 0.0f};
    NeutralMode driveNeutralMode = NeutralMode::Brake;

    DcMotorParams steerMotor = DcMotorParams::krakenX60();
    float steerGearRatio = 150.0f / 7.0f; ///< motor rotations per module rotation
    float steerEfficiency = 0.9f;
    float steerInertia = 0.004f;          ///< kg·m² module about its steer axis, excluding rotor. CALIBRATE
    float steerFrictionTorque = 0.3f;     ///< N·m at the module. CALIBRATE
    CurrentLimits steerCurrentLimits{40.0f, 0.0f};
    NeutralMode steerNeutralMode = NeutralMode::Brake;

    TireParams tire;
    float scrubRadius = 0.01f; ///< m, contact-patch lever arm resisting steering under load. CALIBRATE
};

/// Sensor imperfections. All zero by default (ideal sensors); noise is seeded and deterministic per platform.
struct SensorParams {
    float gyroYawNoise = 0.0f;                   ///< rad, standard deviation of white noise on reported yaw
    float gyroYawDriftRate = 0.0f;               ///< rad/s, constant bias drift since the last reset
    float gyroScaleError = 0.0f;                 ///< fractional, e.g. 0.005 reads 0.5% too much rotation
    std::uint32_t driveEncoderCountsPerRev = 0;  ///< rotor counts per revolution; 0 = continuous
    std::uint32_t seed = 1;
};

/// Very short, stiff suspension standing in for tread and carpet compliance (ADR-0003).
struct SuspensionParams {
    float travel = 0.01f;      ///< m
    float frequency = 15.0f;   ///< Hz
    float dampingRatio = 0.7f;
};

struct SwerveDriveConfig {
    float mass = 60.0f;          ///< kg including bumpers and battery
    float frameHalfX = 0.45f;    ///< m, half bumper-to-bumper length
    float frameHalfY = 0.45f;    ///< m, half bumper-to-bumper width
    float bumperBottom = 0.02f;  ///< m above the carpet
    float bumperHeight = 0.16f;  ///< m, height of the collision box
    float comX = 0.0f;           ///< m, center of mass in the robot frame
    float comY = 0.0f;
    float comHeight = 0.18f;     ///< m above the carpet
    float yawInertia = 0.0f;     ///< kg·m²; 0 = uniform box of `mass`
    MaterialId bumperMaterial = MaterialTable::kDefault;

    std::vector<SwerveModuleConfig> modules;
    SuspensionParams suspension;
    BatteryParams battery;
    SensorParams sensors;
};

inline constexpr std::size_t kMaxSwerveModules = 8;

/// Four modules at (±wheelBase/2, ±trackWidth/2) ordered front-left, front-right, back-left, back-right.
[[nodiscard]] SwerveDriveConfig makeRectangularSwerve(float trackWidth, float wheelBase,
                                                      const SwerveModuleConfig& moduleTemplate = {});

/// Throws std::invalid_argument describing the first invalid value.
void validate(const SwerveDriveConfig& config);

} // namespace frcsim
