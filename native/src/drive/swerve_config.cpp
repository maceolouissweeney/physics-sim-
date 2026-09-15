#include "drive/swerve_config.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace frcsim {
namespace {

bool positive(float v) {
    return std::isfinite(v) && v > 0.0f;
}

bool nonNegative(float v) {
    return std::isfinite(v) && v >= 0.0f;
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::invalid_argument("swerve config: " + message);
    }
}

void validateModule(const SwerveDriveConfig& robot, const SwerveModuleConfig& m, std::size_t i) {
    const std::string at = "modules[" + std::to_string(i) + "]: ";
    require(std::isfinite(m.xMeters) && std::isfinite(m.yMeters), at + "position must be finite");
    require(std::abs(m.xMeters) <= robot.frameHalfXMeters && std::abs(m.yMeters) <= robot.frameHalfYMeters,
            at + "module must be inside the frame footprint");
    require(positive(m.wheelRadiusMeters) && m.wheelRadiusMeters < 0.5f, at + "wheelRadiusMeters must be in (0, 0.5)");
    require(positive(m.wheelWidthMeters), at + "wheelWidthMeters must be > 0");
    require(positive(m.wheelInertiaKgMetersSq), at + "wheelInertiaKgMetersSq must be > 0");
    require(robot.suspension.travelMeters < m.wheelRadiusMeters,
            at + "suspension travel must be smaller than the wheel radius");

    (void)deriveMotorConstants(m.driveMotor);
    require(positive(m.driveGearRatio), at + "driveGearRatio must be > 0");
    require(positive(m.driveEfficiency) && m.driveEfficiency <= 1.0f, at + "driveEfficiency must be in (0, 1]");
    require(nonNegative(m.driveFrictionTorqueNewtonMeters), at + "driveFrictionTorqueNewtonMeters must be >= 0");
    require(nonNegative(m.driveCurrentLimits.statorAmps) && nonNegative(m.driveCurrentLimits.supplyAmps),
            at + "drive current limits must be >= 0");

    (void)deriveMotorConstants(m.steerMotor);
    require(positive(m.steerGearRatio), at + "steerGearRatio must be > 0");
    require(positive(m.steerEfficiency) && m.steerEfficiency <= 1.0f, at + "steerEfficiency must be in (0, 1]");
    require(positive(m.steerInertiaKgMetersSq), at + "steerInertiaKgMetersSq must be > 0");
    require(nonNegative(m.steerFrictionTorqueNewtonMeters), at + "steerFrictionTorqueNewtonMeters must be >= 0");
    require(nonNegative(m.steerCurrentLimits.statorAmps) && nonNegative(m.steerCurrentLimits.supplyAmps),
            at + "steer current limits must be >= 0");
    require(std::isfinite(m.couplingGearRatio), at + "couplingGearRatio must be finite");

    require(positive(m.tire.kineticFriction) && std::isfinite(m.tire.staticFriction) &&
                m.tire.staticFriction >= m.tire.kineticFriction,
            at + "tire friction must satisfy 0 < kinetic <= static");
    require(positive(m.tire.transitionSlipSpeedMetersPerSec), at + "tire transitionSlipSpeedMetersPerSec must be > 0");
    require(nonNegative(m.scrubRadiusMeters), at + "scrubRadiusMeters must be >= 0");
}

} // namespace

SwerveDriveConfig makeRectangularSwerve(float trackWidthMeters, float wheelBaseMeters,
                                        const SwerveModuleConfig& moduleTemplate) {
    SwerveDriveConfig config;
    const float halfWheelBaseMeters = 0.5f * wheelBaseMeters;
    const float halfTrackWidthMeters = 0.5f * trackWidthMeters;
    for (const auto& [xMeters, yMeters] :
         {std::pair{halfWheelBaseMeters, halfTrackWidthMeters}, std::pair{halfWheelBaseMeters, -halfTrackWidthMeters},
          std::pair{-halfWheelBaseMeters, halfTrackWidthMeters}, std::pair{-halfWheelBaseMeters, -halfTrackWidthMeters}}) {
        SwerveModuleConfig module = moduleTemplate;
        module.xMeters = xMeters;
        module.yMeters = yMeters;
        config.modules.push_back(module);
    }
    return config;
}

void validate(const SwerveDriveConfig& c) {
    require(positive(c.massKg), "massKg must be > 0");
    require(positive(c.frameHalfXMeters) && positive(c.frameHalfYMeters), "frame half extents must be > 0");
    require(nonNegative(c.bumperBottomMeters), "bumperBottomMeters must be >= 0");
    require(positive(c.bumperHeightMeters), "bumperHeightMeters must be > 0");
    require(std::isfinite(c.comXMeters) && std::isfinite(c.comYMeters), "center of mass must be finite");
    require(positive(c.comHeightMeters) && c.comHeightMeters < 2.0f, "comHeightMeters must be in (0, 2)");
    require(nonNegative(c.yawInertiaKgMetersSq), "yawInertiaKgMetersSq must be >= 0");
    require(!c.modules.empty() && c.modules.size() <= kMaxSwerveModules, "a swerve drive needs 1..8 modules");
    require(positive(c.suspension.travelMeters), "suspension travelMeters must be > 0");
    require(positive(c.suspension.frequencyHz), "suspension frequencyHz must be > 0");
    require(nonNegative(c.suspension.dampingRatio), "suspension dampingRatio must be >= 0");
    require(nonNegative(c.sensors.gyroYawNoiseRadians), "sensors.gyroYawNoiseRadians must be >= 0");
    require(std::isfinite(c.sensors.gyroYawDriftRateRadPerSec), "sensors.gyroYawDriftRateRadPerSec must be finite");
    require(std::isfinite(c.sensors.gyroScaleError) && c.sensors.gyroScaleError > -1.0f,
            "sensors.gyroScaleError must be finite and > -1");
    Battery::validate(c.battery);
    for (std::size_t i = 0; i < c.modules.size(); ++i) {
        validateModule(c, c.modules[i], i);
    }
}

} // namespace frcsim
