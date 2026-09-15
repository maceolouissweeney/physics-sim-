#include "drive/dc_motor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace frcsim {
namespace {

constexpr float kRadPerSecPerRpm = 2.0f * 3.14159265358979f / 60.0f;

DcMotorParams preset(float stallTorqueNewtonMeters, float stallCurrentAmps, float freeCurrentAmps, float freeSpeedRpm,
                     int count, float rotorInertiaKgMetersSq) {
    DcMotorParams p;
    p.nominalVoltageVolts = 12.0f;
    p.stallTorqueNewtonMeters = stallTorqueNewtonMeters;
    p.stallCurrentAmps = stallCurrentAmps;
    p.freeCurrentAmps = freeCurrentAmps;
    p.freeSpeedRadPerSec = freeSpeedRpm * kRadPerSecPerRpm;
    p.count = count;
    p.rotorInertiaKgMetersSq = rotorInertiaKgMetersSq;
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
DcMotorParams DcMotorParams::minion(int count) {
    return preset(3.17f, 211.0f, 2.0f, 7704.0f, count, 3.5e-5f);
}

DcMotorConstants deriveMotorConstants(const DcMotorParams& p) {
    if (!positiveFinite(p.nominalVoltageVolts) || !positiveFinite(p.stallTorqueNewtonMeters) ||
        !positiveFinite(p.stallCurrentAmps) || !positiveFinite(p.freeSpeedRadPerSec) ||
        !std::isfinite(p.freeCurrentAmps) || p.freeCurrentAmps < 0.0f) {
        throw std::invalid_argument("motor parameters must be finite and positive");
    }
    if (p.count < 1) {
        throw std::invalid_argument("motor count must be >= 1");
    }
    if (!std::isfinite(p.rotorInertiaKgMetersSq) || p.rotorInertiaKgMetersSq < 0.0f) {
        throw std::invalid_argument("motor rotor inertia must be finite and >= 0");
    }
    const auto n = static_cast<float>(p.count);
    const float stallCurrentAmps = p.stallCurrentAmps * n;
    const float freeCurrentAmps = p.freeCurrentAmps * n;
    DcMotorConstants c;
    c.resistanceOhms = p.nominalVoltageVolts / stallCurrentAmps;
    const float backEmfAtFreeSpeedVolts = p.nominalVoltageVolts - c.resistanceOhms * freeCurrentAmps;
    if (!(backEmfAtFreeSpeedVolts > 0.0f)) {
        throw std::invalid_argument("motor free current is inconsistent with stall current");
    }
    c.kvRadPerSecPerVolt = p.freeSpeedRadPerSec / backEmfAtFreeSpeedVolts;
    c.ktNewtonMetersPerAmp = (p.stallTorqueNewtonMeters * n) / stallCurrentAmps;
    c.rotorInertiaKgMetersSq = p.rotorInertiaKgMetersSq * n;
    return c;
}

MotorStepResult stepGearedMotor(const DcMotorConstants& motor, const GearboxParams& gearbox, float inertiaKgMetersSq,
                                float velocityRadPerSec, float commandVolts, float busVolts, NeutralMode neutralMode,
                                const CurrentLimits& limits, float externalTorqueNewtonMeters, float dtSeconds) {
    const float ratio = gearbox.ratio;
    const float efficiency = gearbox.efficiency;
    MotorStepResult result;
    float omegaRadPerSec;

    const bool windingsOpen = (neutralMode == NeutralMode::Coast && commandVolts == 0.0f) || busVolts <= 0.0f;
    if (windingsOpen) {
        omegaRadPerSec = velocityRadPerSec + dtSeconds * externalTorqueNewtonMeters / inertiaKgMetersSq;
    } else {
        const float volts = std::clamp(commandVolts, -busVolts, busVolts);
        // Mechanism torque = drive - damping * omega; solved implicitly because back-EMF damping is stiff.
        const float driveTorqueNewtonMeters = efficiency * ratio * motor.ktNewtonMetersPerAmp * volts / motor.resistanceOhms;
        const float dampingNewtonMeterSecPerRad =
            efficiency * ratio * ratio * motor.ktNewtonMetersPerAmp / (motor.kvRadPerSecPerVolt * motor.resistanceOhms);
        omegaRadPerSec =
            (inertiaKgMetersSq * velocityRadPerSec + dtSeconds * (driveTorqueNewtonMeters + externalTorqueNewtonMeters)) /
            (inertiaKgMetersSq + dtSeconds * dampingNewtonMeterSecPerRad);
        float currentAmps = (volts - ratio * omegaRadPerSec / motor.kvRadPerSecPerVolt) / motor.resistanceOhms;

        float currentLimitAmps = limits.statorAmps > 0.0f ? limits.statorAmps : std::numeric_limits<float>::infinity();
        float appliedVolts = volts;
        if (std::abs(currentAmps) > currentLimitAmps || limits.supplyAmps > 0.0f) {
            // A current-limited controller lowers its duty cycle: the voltage it actually applies is
            // I*R + back-EMF, and supply current follows that effective voltage (decision D28).
            const float backEmfVolts = ratio * velocityRadPerSec / motor.kvRadPerSecPerVolt;
            if (limits.supplyAmps > 0.0f) {
                // |I * (I*R + backEmf) / Vbus| <= supplyLimit, solved for the largest |I| with the current's sign.
                const float sign = currentAmps >= 0.0f ? 1.0f : -1.0f;
                const float emfAlongCurrentVolts = sign * backEmfVolts;
                const float discriminant = emfAlongCurrentVolts * emfAlongCurrentVolts +
                                           4.0f * motor.resistanceOhms * limits.supplyAmps * busVolts;
                const float supplyCapAmps = (-emfAlongCurrentVolts + std::sqrt(std::max(0.0f, discriminant))) /
                                            (2.0f * motor.resistanceOhms);
                currentLimitAmps = std::min(currentLimitAmps, std::max(0.0f, supplyCapAmps));
            }
            if (std::abs(currentAmps) > currentLimitAmps) {
                currentAmps = std::copysign(currentLimitAmps, currentAmps);
                omegaRadPerSec = velocityRadPerSec +
                                 dtSeconds * (efficiency * ratio * motor.ktNewtonMetersPerAmp * currentAmps +
                                              externalTorqueNewtonMeters) /
                                     inertiaKgMetersSq;
                appliedVolts = std::clamp(currentAmps * motor.resistanceOhms + ratio * omegaRadPerSec / motor.kvRadPerSecPerVolt,
                                          -busVolts, busVolts);
            }
        }
        result.appliedVolts = appliedVolts;
        result.statorCurrentAmps = currentAmps;
        result.supplyCurrentAmps = currentAmps * appliedVolts / busVolts;
    }

    // Coulomb friction that can stop the mechanism but never reverse it.
    const float frictionDeltaRadPerSec = gearbox.frictionTorqueNewtonMeters * dtSeconds / inertiaKgMetersSq;
    if (std::abs(omegaRadPerSec) <= frictionDeltaRadPerSec) {
        omegaRadPerSec = 0.0f;
    } else {
        omegaRadPerSec -= std::copysign(frictionDeltaRadPerSec, omegaRadPerSec);
    }
    result.velocityRadPerSec = omegaRadPerSec;
    return result;
}

} // namespace frcsim
