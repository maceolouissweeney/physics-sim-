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
    float xMeters = 0.0f; ///< module center in the robot frame, +X forward
    float yMeters = 0.0f; ///< +Y left

    float wheelRadiusMeters = 0.0508f;       ///< 4 in wheel
    float wheelWidthMeters = 0.038f;
    float wheelInertiaKgMetersSq = 3.0e-4f;  ///< about the axle, excluding the motor rotor. CALIBRATE

    DcMotorParams driveMotor = DcMotorParams::krakenX60();
    float driveGearRatio = 6.75f;                 ///< motor rotations per wheel rotation
    float driveEfficiency = 0.95f;
    float driveFrictionTorqueNewtonMeters = 0.2f; ///< at the wheel. CALIBRATE
    CurrentLimits driveCurrentLimits{80.0f, 0.0f};
    NeutralMode driveNeutralMode = NeutralMode::Brake;

    DcMotorParams steerMotor = DcMotorParams::krakenX60();
    float steerGearRatio = 150.0f / 7.0f;         ///< motor rotations per module rotation
    float steerEfficiency = 0.9f;
    float steerInertiaKgMetersSq = 0.004f;        ///< module about its steer axis, excluding rotor. CALIBRATE
    float steerFrictionTorqueNewtonMeters = 0.3f; ///< at the module. CALIBRATE
    CurrentLimits steerCurrentLimits{40.0f, 0.0f};
    NeutralMode steerNeutralMode = NeutralMode::Brake;

    /// Drive motor rotations caused by one module rotation with the wheel held still (CTRE `CouplingGearRatio`).
    /// Affects the reported drive encoder only.
    float couplingGearRatio = 0.0f;

    TireParams tire;
    float scrubRadiusMeters = 0.01f; ///< contact-patch lever arm resisting steering under load. CALIBRATE
};

/// Very short, stiff suspension standing in for tread and carpet compliance (ADR-0003).
struct SuspensionParams {
    float travelMeters = 0.01f;
    float frequencyHz = 15.0f;
    float dampingRatio = 0.7f;
};

/// Sensor imperfections. All zero by default (ideal sensors); noise is seeded and deterministic per platform.
struct SensorParams {
    float gyroYawNoiseRadians = 0.0f;           ///< standard deviation of white noise on reported yaw
    float gyroYawDriftRateRadPerSec = 0.0f;     ///< constant bias drift since the last reset
    float gyroScaleError = 0.0f;                ///< fractional, e.g. 0.005 reads 0.5% too much rotation
    std::uint32_t driveEncoderCountsPerRev = 0; ///< rotor counts per revolution; 0 = continuous
    std::uint32_t seed = 1;
};

struct SwerveDriveConfig {
    float massKg = 60.0f;                ///< including bumpers and battery
    float frameHalfXMeters = 0.45f;      ///< half bumper-to-bumper length
    float frameHalfYMeters = 0.45f;      ///< half bumper-to-bumper width
    float bumperBottomMeters = 0.02f;    ///< above the carpet
    float bumperHeightMeters = 0.16f;    ///< height of the collision box
    float comXMeters = 0.0f;             ///< center of mass in the robot frame
    float comYMeters = 0.0f;
    float comHeightMeters = 0.18f;       ///< above the carpet
    float yawInertiaKgMetersSq = 0.0f;   ///< 0 = uniform box of massKg
    MaterialId bumperMaterial = MaterialTable::kDefault;

    std::vector<SwerveModuleConfig> modules;
    SuspensionParams suspension;
    BatteryParams battery;
    SensorParams sensors;
};

inline constexpr std::size_t kMaxSwerveModules = 8;

/// Four modules at (±wheelBase/2, ±trackWidth/2) ordered front-left, front-right, back-left, back-right.
[[nodiscard]] SwerveDriveConfig makeRectangularSwerve(float trackWidthMeters, float wheelBaseMeters,
                                                      const SwerveModuleConfig& moduleTemplate = {});

/// Throws std::invalid_argument describing the first invalid value.
void validate(const SwerveDriveConfig& config);

} // namespace frcsim
