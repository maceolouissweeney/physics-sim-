#include "drive/swerve_config.h"

#include <cmath>
#include <stdexcept>
#include <string>

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
    require(std::isfinite(m.x) && std::isfinite(m.y), at + "position must be finite");
    require(std::abs(m.x) <= robot.frameHalfX && std::abs(m.y) <= robot.frameHalfY,
            at + "module must be inside the frame footprint");
    require(positive(m.wheelRadius) && m.wheelRadius < 0.5f, at + "wheelRadius must be in (0, 0.5) m");
    require(positive(m.wheelWidth), at + "wheelWidth must be > 0");
    require(positive(m.wheelInertia), at + "wheelInertia must be > 0");
    require(robot.suspension.travel < m.wheelRadius, at + "suspension travel must be smaller than the wheel radius");

    (void)deriveMotorConstants(m.driveMotor);
    require(positive(m.driveGearRatio), at + "driveGearRatio must be > 0");
    require(positive(m.driveEfficiency) && m.driveEfficiency <= 1.0f, at + "driveEfficiency must be in (0, 1]");
    require(nonNegative(m.driveFrictionTorque), at + "driveFrictionTorque must be >= 0");
    require(nonNegative(m.driveCurrentLimits.stator) && nonNegative(m.driveCurrentLimits.supply),
            at + "drive current limits must be >= 0");

    (void)deriveMotorConstants(m.steerMotor);
    require(positive(m.steerGearRatio), at + "steerGearRatio must be > 0");
    require(positive(m.steerEfficiency) && m.steerEfficiency <= 1.0f, at + "steerEfficiency must be in (0, 1]");
    require(positive(m.steerInertia), at + "steerInertia must be > 0");
    require(nonNegative(m.steerFrictionTorque), at + "steerFrictionTorque must be >= 0");
    require(nonNegative(m.steerCurrentLimits.stator) && nonNegative(m.steerCurrentLimits.supply),
            at + "steer current limits must be >= 0");

    require(positive(m.tire.kineticFriction) && m.tire.staticFriction >= m.tire.kineticFriction &&
                std::isfinite(m.tire.staticFriction),
            at + "tire friction must satisfy 0 < kinetic <= static");
    require(positive(m.tire.transitionSlipSpeed), at + "tire transitionSlipSpeed must be > 0");
    require(nonNegative(m.scrubRadius), at + "scrubRadius must be >= 0");
}

} // namespace

SwerveDriveConfig makeRectangularSwerve(float trackWidth, float wheelBase, const SwerveModuleConfig& moduleTemplate) {
    SwerveDriveConfig config;
    const float hx = 0.5f * wheelBase;
    const float hy = 0.5f * trackWidth;
    for (const auto& [x, y] : {std::pair{hx, hy}, std::pair{hx, -hy}, std::pair{-hx, hy}, std::pair{-hx, -hy}}) {
        SwerveModuleConfig module = moduleTemplate;
        module.x = x;
        module.y = y;
        config.modules.push_back(module);
    }
    return config;
}

void validate(const SwerveDriveConfig& c) {
    require(positive(c.mass), "mass must be > 0");
    require(positive(c.frameHalfX) && positive(c.frameHalfY), "frame half extents must be > 0");
    require(nonNegative(c.bumperBottom), "bumperBottom must be >= 0");
    require(positive(c.bumperHeight), "bumperHeight must be > 0");
    require(std::isfinite(c.comX) && std::isfinite(c.comY), "center of mass must be finite");
    require(positive(c.comHeight) && c.comHeight < 2.0f, "comHeight must be in (0, 2) m");
    require(nonNegative(c.yawInertia), "yawInertia must be >= 0");
    require(!c.modules.empty() && c.modules.size() <= kMaxSwerveModules, "a swerve drive needs 1..8 modules");
    require(positive(c.suspension.travel), "suspension travel must be > 0");
    require(positive(c.suspension.frequency), "suspension frequency must be > 0");
    require(nonNegative(c.suspension.dampingRatio), "suspension dampingRatio must be >= 0");
    require(nonNegative(c.sensors.gyroYawNoise), "sensors.gyroYawNoise must be >= 0");
    require(std::isfinite(c.sensors.gyroYawDriftRate), "sensors.gyroYawDriftRate must be finite");
    require(std::isfinite(c.sensors.gyroScaleError) && c.sensors.gyroScaleError > -1.0f,
            "sensors.gyroScaleError must be finite and > -1");
    Battery::validate(c.battery);
    for (std::size_t i = 0; i < c.modules.size(); ++i) {
        validateModule(c, c.modules[i], i);
    }
}

} // namespace frcsim
