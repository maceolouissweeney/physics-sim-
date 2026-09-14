#pragma once

#include <cstdint>

namespace frcsim {

/// DC motor parameters in WPILib `DCMotor` form (see docs/models/swerve.md).
struct DcMotorParams {
    float nominalVoltage = 12.0f; ///< V
    float stallTorque = 0.0f;     ///< N·m, one motor
    float stallCurrent = 0.0f;    ///< A, one motor
    float freeCurrent = 0.0f;     ///< A, one motor
    float freeSpeed = 0.0f;       ///< rad/s
    int count = 1;                ///< identical motors driving one gearbox
    float rotorInertia = 6.0e-5f; ///< kg·m² per motor. CALIBRATE: not published by vendors.

    // Presets with WPILib 2026 constants. Rotor inertias are estimates (CALIBRATE).
    [[nodiscard]] static DcMotorParams krakenX60(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX60Foc(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX44(int count = 1);
    [[nodiscard]] static DcMotorParams krakenX44Foc(int count = 1);
    [[nodiscard]] static DcMotorParams falcon500(int count = 1);
    [[nodiscard]] static DcMotorParams falcon500Foc(int count = 1);
    [[nodiscard]] static DcMotorParams neo(int count = 1);
    [[nodiscard]] static DcMotorParams neoVortex(int count = 1);
};

/// Electrical constants for all `count` motors combined.
struct DcMotorConstants {
    float resistance = 0.0f;   ///< Ω
    float kv = 0.0f;           ///< rad/s per V
    float kt = 0.0f;           ///< N·m per A
    float rotorInertia = 0.0f; ///< kg·m², all rotors
};

/// Derives constants exactly like WPILib `DCMotor`. Throws std::invalid_argument for invalid parameters.
[[nodiscard]] DcMotorConstants deriveMotorConstants(const DcMotorParams& params);

/// Current limits in amps; 0 disables a limit.
struct CurrentLimits {
    float stator = 0.0f;
    float supply = 0.0f;
};

enum class NeutralMode : std::uint8_t {
    Brake = 0, ///< windings shorted at 0 V: back-EMF braking
    Coast = 1, ///< windings open at 0 V: no current
};

struct GearboxParams {
    float ratio = 1.0f;          ///< motor rotations per mechanism rotation
    float efficiency = 1.0f;     ///< 0..1, applied to motor torque
    float frictionTorque = 0.0f; ///< N·m Coulomb friction at the mechanism
};

struct MotorStepResult {
    float velocity = 0.0f;       ///< mechanism angular velocity at the end of the step (rad/s)
    float appliedVoltage = 0.0f; ///< V
    float statorCurrent = 0.0f;  ///< A, all motors
    float supplyCurrent = 0.0f;  ///< A, all motors (signed; negative when regenerating)
};

/**
 * Advances a motor-driven mechanism by dt with implicit back-EMF damping, current limits, neutral mode, and
 * overshoot-free Coulomb friction. `inertia` is the total mechanism-side inertia (including reflected rotor
 * inertia); `externalTorque` acts on the mechanism. Does not allocate.
 */
[[nodiscard]] MotorStepResult stepGearedMotor(const DcMotorConstants& motor, const GearboxParams& gearbox, float inertia,
                                              float velocity, float commandVoltage, float busVoltage,
                                              NeutralMode neutralMode, const CurrentLimits& limits,
                                              float externalTorque, float dt);

} // namespace frcsim
