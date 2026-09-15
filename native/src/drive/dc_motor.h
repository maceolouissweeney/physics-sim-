#pragma once

#include <cstdint>

namespace frcsim {

/// DC motor parameters in WPILib `DCMotor` form (see docs/models/swerve.md).
struct DcMotorParams {
    float nominalVoltageVolts = 12.0f;
    float stallTorqueNewtonMeters = 0.0f;    ///< one motor
    float stallCurrentAmps = 0.0f;           ///< one motor
    float freeCurrentAmps = 0.0f;            ///< one motor
    float freeSpeedRadPerSec = 0.0f;
    int count = 1;                           ///< identical motors driving one gearbox
    float rotorInertiaKgMetersSq = 6.0e-5f;  ///< per motor. CALIBRATE: not published by vendors.

    // CTRE motor presets with WPILib 2026 constants. Rotor inertias are estimates (CALIBRATE).
    [[nodiscard]] static DcMotorParams krakenX60(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX60Foc(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX44(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX44Foc(int count = 1);
    [[nodiscard]] static DcMotorParams falcon500(int count = 1);
    [[nodiscard]] static DcMotorParams falcon500Foc(int count = 1);
    [[nodiscard]] static DcMotorParams minion(int count = 1);
};

/// Electrical constants for all `count` motors combined.
struct DcMotorConstants {
    float resistanceOhms = 0.0f;
    float kvRadPerSecPerVolt = 0.0f;
    float ktNewtonMetersPerAmp = 0.0f;
    float rotorInertiaKgMetersSq = 0.0f; ///< all rotors
};

/// Derives constants exactly like WPILib `DCMotor`. Throws std::invalid_argument for invalid parameters.
[[nodiscard]] DcMotorConstants deriveMotorConstants(const DcMotorParams& params);

/// Current limits; 0 disables a limit.
struct CurrentLimits {
    float statorAmps = 0.0f;
    float supplyAmps = 0.0f;
};

enum class NeutralMode : std::uint8_t {
    Brake = 0, ///< windings shorted at 0 V: back-EMF braking
    Coast = 1, ///< windings open at 0 V: no current
};

struct GearboxParams {
    float ratio = 1.0f;                      ///< motor rotations per mechanism rotation
    float efficiency = 1.0f;                 ///< 0..1, applied to motor torque
    float frictionTorqueNewtonMeters = 0.0f; ///< Coulomb friction at the mechanism
};

struct MotorStepResult {
    float velocityRadPerSec = 0.0f; ///< mechanism angular velocity at the end of the step
    float appliedVolts = 0.0f;      ///< effective voltage (lower than commanded under current limiting)
    float statorCurrentAmps = 0.0f; ///< all motors
    float supplyCurrentAmps = 0.0f; ///< all motors (signed; negative when regenerating)
};

/**
 * Advances a motor-driven mechanism by dtSeconds with implicit back-EMF damping, current limits, neutral
 * mode, and overshoot-free Coulomb friction. `inertiaKgMetersSq` is the total mechanism-side inertia
 * (including reflected rotor inertia); `externalTorqueNewtonMeters` acts on the mechanism. Does not allocate.
 */
[[nodiscard]] MotorStepResult stepGearedMotor(const DcMotorConstants& motor, const GearboxParams& gearbox,
                                              float inertiaKgMetersSq, float velocityRadPerSec, float commandVolts,
                                              float busVolts, NeutralMode neutralMode, const CurrentLimits& limits,
                                              float externalTorqueNewtonMeters, float dtSeconds);

} // namespace frcsim
