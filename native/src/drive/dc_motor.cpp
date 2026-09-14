#include "drive/dc_motor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace frcsim {
namespace {

constexpr float kRpmToRadPerSec = 2.0f * 3.14159265358979f / 60.0f;

DcMotorParams preset(float stallTorque, float stallCurrent, float freeCurrent, float freeSpeedRpm, int count,
                     float rotorInertia) {
    DcMotorParams p;
    p.nominalVoltage = 12.0f;
    p.stallTorque = stallTorque;
    p.stallCurrent = stallCurrent;
    p.freeCurrent = freeCurrent;
    p.freeSpeed = freeSpeedRpm * kRpmToRadPerSec;
    p.count = count;
    p.rotorInertia = rotorInertia;
    return p;
}

bool positiveFinite(float value) {
    return std::isfinite(value) && value > 0.0f;
}

} // namespace

// Constants from WPILib 2026 DCMotor.java. Rotor inertias: CALIBRATE (estimates from rotor size).
DcMotorParams DcMotorParams::krakenX60(int count) {
    return preset(7.09f, 366.0f, 2.0f, 6000.0f, count, 6.0e-5f);
}
DcMotorParams DcMotorParams::krakenX60Foc(int count) {
    return preset(9.37f, 483.0f, 2.0f, 5800.0f, count, 6.0e-5f);
}
DcMotorParams DcMotorParams::krakenX44(int count) {
    return preset(4.11f, 279.0f, 2.0f, 7758.0f, count, 3.0e-5f);
}
DcMotorParams DcMotorParams::krakenX44Foc(int count) {
    return preset(5.01f, 329.0f, 2.0f, 7368.0f, count, 3.0e-5f);
}
DcMotorParams DcMotorParams::falcon500(int count) {
    return preset(4.69f, 257.0f, 1.5f, 6380.0f, count, 5.5e-5f);
}
DcMotorParams DcMotorParams::falcon500Foc(int count) {
    return preset(5.84f, 304.0f, 1.5f, 6080.0f, count, 5.5e-5f);
}
DcMotorParams DcMotorParams::neo(int count) {
    return preset(2.6f, 105.0f, 1.8f, 5676.0f, count, 4.0e-5f);
}
DcMotorParams DcMotorParams::neoVortex(int count) {
    return preset(3.60f, 211.0f, 3.6f, 6784.0f, count, 5.0e-5f);
}

DcMotorConstants deriveMotorConstants(const DcMotorParams& p) {
    if (!positiveFinite(p.nominalVoltage) || !positiveFinite(p.stallTorque) || !positiveFinite(p.stallCurrent) ||
        !positiveFinite(p.freeSpeed) || !std::isfinite(p.freeCurrent) || p.freeCurrent < 0.0f) {
        throw std::invalid_argument("motor parameters must be finite and positive");
    }
    if (p.count < 1) {
        throw std::invalid_argument("motor count must be >= 1");
    }
    if (!std::isfinite(p.rotorInertia) || p.rotorInertia < 0.0f) {
        throw std::invalid_argument("motor rotor inertia must be finite and >= 0");
    }
    const auto n = static_cast<float>(p.count);
    const float stallCurrent = p.stallCurrent * n;
    const float freeCurrent = p.freeCurrent * n;
    DcMotorConstants c;
    c.resistance = p.nominalVoltage / stallCurrent;
    const float backEmfAtFree = p.nominalVoltage - c.resistance * freeCurrent;
    if (!(backEmfAtFree > 0.0f)) {
        throw std::invalid_argument("motor free current is inconsistent with stall current");
    }
    c.kv = p.freeSpeed / backEmfAtFree;
    c.kt = (p.stallTorque * n) / stallCurrent;
    c.rotorInertia = p.rotorInertia * n;
    return c;
}

MotorStepResult stepGearedMotor(const DcMotorConstants& motor, const GearboxParams& gearbox, float inertia,
                                float velocity, float commandVoltage, float busVoltage, NeutralMode neutralMode,
                                const CurrentLimits& limits, float externalTorque, float dt) {
    const float g = gearbox.ratio;
    const float eta = gearbox.efficiency;
    MotorStepResult result;
    float omega;

    const bool windingsOpen = (neutralMode == NeutralMode::Coast && commandVoltage == 0.0f) || busVoltage <= 0.0f;
    if (windingsOpen) {
        omega = velocity + dt * externalTorque / inertia;
    } else {
        const float voltage = std::clamp(commandVoltage, -busVoltage, busVoltage);
        // Mechanism torque = A - B * omega; solve implicitly because B (back-EMF) is stiff.
        const float a = eta * g * motor.kt * voltage / motor.resistance;
        const float b = eta * g * g * motor.kt / (motor.kv * motor.resistance);
        omega = (inertia * velocity + dt * (a + externalTorque)) / (inertia + dt * b);
        float current = (voltage - g * omega / motor.kv) / motor.resistance;

        float currentLimit = limits.stator > 0.0f ? limits.stator : std::numeric_limits<float>::infinity();
        float appliedVoltage = voltage;
        if (std::abs(current) > currentLimit || limits.supply > 0.0f) {
            // A current-limited controller lowers its duty cycle: the voltage it actually applies is
            // I*R + back-EMF. Supply current follows that effective voltage, not the command.
            const float backEmf = g * velocity / motor.kv;
            if (limits.supply > 0.0f) {
                // |I * (I*R + backEmf) / Vbus| <= supplyLimit, solved for the largest |I| with the current's sign.
                const float s = current >= 0.0f ? 1.0f : -1.0f;
                const float emf = s * backEmf; // back-EMF component along the current's direction
                const float disc = emf * emf + 4.0f * motor.resistance * limits.supply * busVoltage;
                const float supplyCap = (-emf + std::sqrt(std::max(0.0f, disc))) / (2.0f * motor.resistance);
                currentLimit = std::min(currentLimit, std::max(0.0f, supplyCap));
            }
            if (std::abs(current) > currentLimit) {
                current = std::copysign(currentLimit, current);
                omega = velocity + dt * (eta * g * motor.kt * current + externalTorque) / inertia;
                appliedVoltage = std::clamp(current * motor.resistance + g * omega / motor.kv, -busVoltage, busVoltage);
            }
        }
        result.appliedVoltage = appliedVoltage;
        result.statorCurrent = current;
        result.supplyCurrent = current * appliedVoltage / busVoltage;
    }

    // Coulomb friction that can stop the mechanism but never reverse it.
    const float frictionDelta = gearbox.frictionTorque * dt / inertia;
    if (std::abs(omega) <= frictionDelta) {
        omega = 0.0f;
    } else {
        omega -= std::copysign(frictionDelta, omega);
    }
    result.velocity = omega;
    return result;
}

} // namespace frcsim
