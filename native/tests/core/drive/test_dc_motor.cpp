#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drive/battery.h"
#include "drive/dc_motor.h"
#include "drive/tire_model.h"

namespace frcsim {
namespace {

constexpr float kDtSeconds = 0.004f;

TEST(DcMotor, DerivedConstantsMatchWpilib) {
    const DcMotorConstants one = deriveMotorConstants(DcMotorParams::krakenX60());
    EXPECT_NEAR(one.resistanceOhms, 12.0f / 366.0f, 1e-6f);
    EXPECT_NEAR(one.ktNewtonMetersPerAmp, 7.09f / 366.0f, 1e-6f);
    const float freeSpeedRadPerSec = 6000.0f * 2.0f * 3.14159265f / 60.0f;
    EXPECT_NEAR(one.kvRadPerSecPerVolt, freeSpeedRadPerSec / (12.0f - one.resistanceOhms * 2.0f), 1e-3f);

    const DcMotorConstants two = deriveMotorConstants(DcMotorParams::krakenX60(2));
    EXPECT_NEAR(two.resistanceOhms, 12.0f / 732.0f, 1e-6f);
    EXPECT_NEAR(two.ktNewtonMetersPerAmp, one.ktNewtonMetersPerAmp, 1e-6f)
        << "Kt is per amp; torque and current both scale";
    EXPECT_NEAR(two.rotorInertiaKgMetersSq, 2.0f * one.rotorInertiaKgMetersSq, 1e-9f);

    const DcMotorConstants minion = deriveMotorConstants(DcMotorParams::minion());
    EXPECT_NEAR(minion.ktNewtonMetersPerAmp, 3.17f / 211.0f, 1e-6f);

    DcMotorParams bad = DcMotorParams::minion();
    bad.count = 0;
    EXPECT_THROW((void)deriveMotorConstants(bad), std::invalid_argument);
}

TEST(DcMotor, UnloadedMechanismReachesFreeSpeedWithoutOvershoot) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    const float freeSpeedRadPerSec = 12.0f * m.kvRadPerSecPerVolt; // no load, ideal
    float omegaRadPerSec = 0.0f;
    float previousRadPerSec = 0.0f;
    // Tiny inertia + large step: explicit Euler would explode; implicit must approach monotonically.
    for (int i = 0; i < 200; ++i) {
        omegaRadPerSec = stepGearedMotor(m, {1.0f, 1.0f, 0.0f}, 1.0e-6f, omegaRadPerSec, 12.0f, 12.0f,
                                         NeutralMode::Brake, {}, 0.0f, 0.02f)
                             .velocityRadPerSec;
        ASSERT_GE(omegaRadPerSec, previousRadPerSec - 1e-3f);
        ASSERT_LE(omegaRadPerSec, freeSpeedRadPerSec * 1.0001f);
        previousRadPerSec = omegaRadPerSec;
    }
    EXPECT_NEAR(omegaRadPerSec, freeSpeedRadPerSec, freeSpeedRadPerSec * 1e-3f);
}

TEST(DcMotor, StallCurrentIsVoltageOverResistance) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    const MotorStepResult r =
        stepGearedMotor(m, {6.75f, 1.0f, 0.0f}, 1.0e6f, 0.0f, 12.0f, 12.0f, NeutralMode::Brake, {}, 0.0f, kDtSeconds);
    EXPECT_NEAR(r.statorCurrentAmps, 366.0f, 366.0f * 0.005f);
    EXPECT_NEAR(r.supplyCurrentAmps, r.statorCurrentAmps, 1e-3f) << "100% duty: supply equals stator";
}

TEST(DcMotor, StatorLimitClampsCurrentAndTorque) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    constexpr float kInertiaKgMetersSq = 0.01f;
    const MotorStepResult r = stepGearedMotor(m, {6.75f, 0.9f, 0.0f}, kInertiaKgMetersSq, 0.0f, 12.0f, 12.0f,
                                              NeutralMode::Brake, CurrentLimits{40.0f, 0.0f}, 0.0f, kDtSeconds);
    EXPECT_FLOAT_EQ(r.statorCurrentAmps, 40.0f);
    EXPECT_NEAR(r.velocityRadPerSec, kDtSeconds * 0.9f * 6.75f * m.ktNewtonMetersPerAmp * 40.0f / kInertiaKgMetersSq,
                1e-4f);
}

TEST(DcMotor, StatorLimitedMotorDrawsLessFromSupply) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    // Stalled at a 40 A stator limit, the controller only applies I*R volts: supply = 40 * (40 R) / 12.
    const MotorStepResult r = stepGearedMotor(m, {1.0f, 1.0f, 0.0f}, 1.0e6f, 0.0f, 12.0f, 12.0f, NeutralMode::Brake,
                                              CurrentLimits{40.0f, 0.0f}, 0.0f, kDtSeconds);
    EXPECT_FLOAT_EQ(r.statorCurrentAmps, 40.0f);
    EXPECT_NEAR(r.appliedVolts, 40.0f * m.resistanceOhms, 1e-3f);
    EXPECT_NEAR(r.supplyCurrentAmps, 40.0f * 40.0f * m.resistanceOhms / 12.0f, 1e-2f);
}

TEST(DcMotor, SupplyLimitBoundsBatteryCurrent) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    // Stalled, unlimited stator: a 30 A supply limit allows I with I * (I*R) / 12 = 30 -> I = sqrt(360 / R).
    const MotorStepResult r = stepGearedMotor(m, {1.0f, 1.0f, 0.0f}, 1.0e6f, 0.0f, 12.0f, 12.0f, NeutralMode::Brake,
                                              CurrentLimits{0.0f, 30.0f}, 0.0f, kDtSeconds);
    EXPECT_NEAR(r.supplyCurrentAmps, 30.0f, 0.05f);
    EXPECT_NEAR(r.statorCurrentAmps, std::sqrt(360.0f / m.resistanceOhms), 0.5f);
    EXPECT_GT(r.statorCurrentAmps, 30.0f) << "at low duty cycle stator current exceeds supply current";
}

TEST(DcMotor, NeutralModes) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    const MotorStepResult coast =
        stepGearedMotor(m, {6.75f, 1.0f, 0.0f}, 0.01f, 50.0f, 0.0f, 12.0f, NeutralMode::Coast, {}, 0.0f, kDtSeconds);
    EXPECT_FLOAT_EQ(coast.velocityRadPerSec, 50.0f);
    EXPECT_FLOAT_EQ(coast.statorCurrentAmps, 0.0f);

    const MotorStepResult brake =
        stepGearedMotor(m, {6.75f, 1.0f, 0.0f}, 0.01f, 50.0f, 0.0f, 12.0f, NeutralMode::Brake, {}, 0.0f, kDtSeconds);
    EXPECT_LT(brake.velocityRadPerSec, 50.0f);
    EXPECT_GE(brake.velocityRadPerSec, 0.0f) << "implicit braking never reverses";
    EXPECT_LT(brake.statorCurrentAmps, 0.0f);
}

TEST(DcMotor, FrictionHoldsAgainstSmallTorqueAndNeverReverses) {
    const DcMotorConstants m = deriveMotorConstants(DcMotorParams::krakenX60());
    // 0.05 V produces far less torque than 5 N·m of friction.
    const MotorStepResult held =
        stepGearedMotor(m, {6.75f, 1.0f, 5.0f}, 0.01f, 0.0f, 0.05f, 12.0f, NeutralMode::Brake, {}, 0.0f, kDtSeconds);
    EXPECT_FLOAT_EQ(held.velocityRadPerSec, 0.0f);
    const MotorStepResult coasting =
        stepGearedMotor(m, {6.75f, 1.0f, 5.0f}, 0.01f, 0.5f, 0.0f, 12.0f, NeutralMode::Coast, {}, 0.0f, kDtSeconds);
    EXPECT_FLOAT_EQ(coasting.velocityRadPerSec, 0.0f);
}

TEST(Battery, SagAndBrownoutHysteresis) {
    Battery battery(BatteryParams{12.5f, 0.02f, 6.75f, 7.5f});
    battery.update(100.0f);
    EXPECT_NEAR(battery.voltageVolts(), 10.5f, 1e-4f);
    EXPECT_FALSE(battery.brownout());
    battery.update(300.0f); // 6.5 V
    EXPECT_TRUE(battery.brownout());
    battery.update(260.0f); // 7.3 V: still below recovery
    EXPECT_TRUE(battery.brownout());
    battery.update(0.0f);
    EXPECT_FALSE(battery.brownout());
    EXPECT_THROW(Battery(BatteryParams{12.5f, -1.0f, 6.75f, 7.5f}), std::invalid_argument);
}

TEST(TireModel, StaticToKineticTransition) {
    const TireParams tire{1.1f, 0.9f, 0.1f};
    EXPECT_FLOAT_EQ(tireFriction(tire, 0.0f), 1.1f);
    EXPECT_NEAR(tireFriction(tire, 1.0f), 0.9f, 1e-4f);
    EXPECT_GT(tireFriction(tire, 0.05f), tireFriction(tire, 0.15f));
}

} // namespace
} // namespace frcsim
