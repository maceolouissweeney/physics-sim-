package org.frcsim;

import java.util.Objects;

/**
 * Configuration of one swerve module, in the style of CTRE Phoenix 6 configs: public fields with
 * defaults, validated when the robot is added. Defaults describe an SDS MK4i L2 with Kraken X60
 * drive and steer motors. Values marked "estimate" should be calibrated (docs/models/swerve.md).
 */
public final class SwerveModuleConfig {
  /** Module center in the robot frame, +X forward (m). */
  public double x;

  /** Module center in the robot frame, +Y left (m). */
  public double y;

  /** Wheel radius (m). */
  public double wheelRadius = 0.0508;

  /** Wheel width (m). */
  public double wheelWidth = 0.038;

  /** Wheel inertia about its axle, excluding the motor rotor (kg·m²). Estimate. */
  public double wheelInertia = 3.0e-4;

  /** Drive motor(s). */
  public DcMotorSpec driveMotor = DcMotorSpec.krakenX60(1);

  /** Motor rotations per wheel rotation. */
  public double driveGearRatio = 6.75;

  /** Drive gearbox efficiency, 0..1. */
  public double driveEfficiency = 0.95;

  /** Coulomb friction at the wheel (N·m). Estimate. */
  public double driveFrictionTorque = 0.2;

  /** Drive stator current limit (A), 0 for none. */
  public double driveStatorCurrentLimit = 80;

  /** Drive supply current limit (A), 0 for none. */
  public double driveSupplyCurrentLimit = 0;

  /** Drive neutral mode. */
  public NeutralMode driveNeutralMode = NeutralMode.BRAKE;

  /** Steer motor(s). */
  public DcMotorSpec steerMotor = DcMotorSpec.krakenX60(1);

  /** Motor rotations per module rotation. */
  public double steerGearRatio = 150.0 / 7.0;

  /** Steer gearbox efficiency, 0..1. */
  public double steerEfficiency = 0.9;

  /** Module inertia about the steer axis, excluding the motor rotor (kg·m²). Estimate. */
  public double steerInertia = 0.004;

  /** Coulomb friction about the steer axis (N·m). Estimate. */
  public double steerFrictionTorque = 0.3;

  /** Steer stator current limit (A), 0 for none. */
  public double steerStatorCurrentLimit = 40;

  /** Steer supply current limit (A), 0 for none. */
  public double steerSupplyCurrentLimit = 0;

  /** Steer neutral mode. */
  public NeutralMode steerNeutralMode = NeutralMode.BRAKE;

  /** Tire friction coefficient at zero slip. Estimate; measure with a pull test. */
  public double tireStaticFriction = 1.1;

  /** Tire friction coefficient while sliding. Estimate. */
  public double tireKineticFriction = 0.9;

  /** Slip speed over which friction transitions from static to kinetic (m/s). Estimate. */
  public double tireTransitionSlipSpeed = 0.1;

  /** Contact-patch lever arm resisting steering under load (m). Estimate. */
  public double scrubRadius = 0.01;

  /** Values per module in the packed JNI representation. */
  static final int PACKED_SIZE = 36;

  /**
   * Sets the module position.
   *
   * @param x module center, +X forward (m)
   * @param y module center, +Y left (m)
   * @return this config
   */
  public SwerveModuleConfig withPosition(double x, double y) {
    this.x = x;
    this.y = y;
    return this;
  }

  /**
   * Returns an independent copy.
   *
   * @return copy
   */
  public SwerveModuleConfig copy() {
    SwerveModuleConfig c = new SwerveModuleConfig();
    c.x = x;
    c.y = y;
    c.wheelRadius = wheelRadius;
    c.wheelWidth = wheelWidth;
    c.wheelInertia = wheelInertia;
    c.driveMotor = driveMotor;
    c.driveGearRatio = driveGearRatio;
    c.driveEfficiency = driveEfficiency;
    c.driveFrictionTorque = driveFrictionTorque;
    c.driveStatorCurrentLimit = driveStatorCurrentLimit;
    c.driveSupplyCurrentLimit = driveSupplyCurrentLimit;
    c.driveNeutralMode = driveNeutralMode;
    c.steerMotor = steerMotor;
    c.steerGearRatio = steerGearRatio;
    c.steerEfficiency = steerEfficiency;
    c.steerInertia = steerInertia;
    c.steerFrictionTorque = steerFrictionTorque;
    c.steerStatorCurrentLimit = steerStatorCurrentLimit;
    c.steerSupplyCurrentLimit = steerSupplyCurrentLimit;
    c.steerNeutralMode = steerNeutralMode;
    c.tireStaticFriction = tireStaticFriction;
    c.tireKineticFriction = tireKineticFriction;
    c.tireTransitionSlipSpeed = tireTransitionSlipSpeed;
    c.scrubRadius = scrubRadius;
    return c;
  }

  /** Packing order must match {@code ModuleParam} in {@code native/src/jni/frcsim_jni.cpp}. */
  void packInto(float[] out, int offset) {
    Objects.requireNonNull(driveMotor, "driveMotor");
    Objects.requireNonNull(steerMotor, "steerMotor");
    Objects.requireNonNull(driveNeutralMode, "driveNeutralMode");
    Objects.requireNonNull(steerNeutralMode, "steerNeutralMode");
    out[offset] = (float) x;
    out[offset + 1] = (float) y;
    out[offset + 2] = (float) wheelRadius;
    out[offset + 3] = (float) wheelWidth;
    out[offset + 4] = (float) wheelInertia;
    driveMotor.packInto(out, offset + 5);
    out[offset + 12] = (float) driveGearRatio;
    out[offset + 13] = (float) driveEfficiency;
    out[offset + 14] = (float) driveFrictionTorque;
    out[offset + 15] = (float) driveStatorCurrentLimit;
    out[offset + 16] = (float) driveSupplyCurrentLimit;
    out[offset + 17] = driveNeutralMode.ordinal();
    steerMotor.packInto(out, offset + 18);
    out[offset + 25] = (float) steerGearRatio;
    out[offset + 26] = (float) steerEfficiency;
    out[offset + 27] = (float) steerInertia;
    out[offset + 28] = (float) steerFrictionTorque;
    out[offset + 29] = (float) steerStatorCurrentLimit;
    out[offset + 30] = (float) steerSupplyCurrentLimit;
    out[offset + 31] = steerNeutralMode.ordinal();
    out[offset + 32] = (float) tireStaticFriction;
    out[offset + 33] = (float) tireKineticFriction;
    out[offset + 34] = (float) tireTransitionSlipSpeed;
    out[offset + 35] = (float) scrubRadius;
  }
}
