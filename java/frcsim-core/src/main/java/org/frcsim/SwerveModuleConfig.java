package org.frcsim;

import java.util.Objects;

/**
 * Configuration of one swerve module, in the style of CTRE Phoenix 6 configs: public fields with
 * defaults, validated when the robot is added. Defaults describe an SDS MK4i L2 with Kraken X60
 * drive and steer motors. Values marked "estimate" should be calibrated (docs/models/swerve.md).
 */
public final class SwerveModuleConfig {
  /** Module center in the robot frame, +X forward. */
  public double xMeters;

  /** Module center in the robot frame, +Y left. */
  public double yMeters;

  /** Wheel radius. */
  public double wheelRadiusMeters = 0.0508;

  /** Wheel width. */
  public double wheelWidthMeters = 0.038;

  /** Wheel inertia about its axle, excluding the motor rotor. Estimate. */
  public double wheelInertiaKgMetersSq = 3.0e-4;

  /** Drive motor(s). */
  public DcMotorSpec driveMotor = DcMotorSpec.krakenX60(1);

  /** Motor rotations per wheel rotation. */
  public double driveGearRatio = 6.75;

  /** Drive gearbox efficiency, 0..1. */
  public double driveEfficiency = 0.95;

  /** Coulomb friction at the wheel. Estimate. */
  public double driveFrictionTorqueNewtonMeters = 0.2;

  /** Drive stator current limit, 0 for none. */
  public double driveStatorCurrentLimitAmps = 80;

  /** Drive supply current limit, 0 for none. */
  public double driveSupplyCurrentLimitAmps = 0;

  /** Drive neutral mode. */
  public NeutralMode driveNeutralMode = NeutralMode.BRAKE;

  /** Steer motor(s). */
  public DcMotorSpec steerMotor = DcMotorSpec.krakenX60(1);

  /** Motor rotations per module rotation. */
  public double steerGearRatio = 150.0 / 7.0;

  /** Steer gearbox efficiency, 0..1. */
  public double steerEfficiency = 0.9;

  /** Module inertia about the steer axis, excluding the motor rotor. Estimate. */
  public double steerInertiaKgMetersSq = 0.004;

  /** Coulomb friction about the steer axis. Estimate. */
  public double steerFrictionTorqueNewtonMeters = 0.3;

  /** Steer stator current limit, 0 for none. */
  public double steerStatorCurrentLimitAmps = 40;

  /** Steer supply current limit, 0 for none. */
  public double steerSupplyCurrentLimitAmps = 0;

  /** Steer neutral mode. */
  public NeutralMode steerNeutralMode = NeutralMode.BRAKE;

  /**
   * Drive motor rotations caused by one module rotation with the wheel held still (CTRE {@code
   * CouplingGearRatio}). Affects the reported drive rotor position and velocity only.
   */
  public double couplingGearRatio = 0;

  /** Tire friction coefficient at zero slip. Estimate; measure with a pull test. */
  public double tireStaticFriction = 1.1;

  /** Tire friction coefficient while sliding. Estimate. */
  public double tireKineticFriction = 0.9;

  /** Slip speed over which friction transitions from static to kinetic. Estimate. */
  public double tireTransitionSlipSpeedMetersPerSec = 0.1;

  /** Contact-patch lever arm resisting steering under load. Estimate. */
  public double scrubRadiusMeters = 0.01;

  /** Values per module in the packed JNI representation. */
  static final int PACKED_SIZE = 37;

  /**
   * Sets the module position.
   *
   * @param xMeters module center, +X forward
   * @param yMeters module center, +Y left
   * @return this config
   */
  public SwerveModuleConfig withPosition(double xMeters, double yMeters) {
    this.xMeters = xMeters;
    this.yMeters = yMeters;
    return this;
  }

  /**
   * Returns an independent copy.
   *
   * @return copy
   */
  public SwerveModuleConfig copy() {
    SwerveModuleConfig c = new SwerveModuleConfig();
    c.xMeters = xMeters;
    c.yMeters = yMeters;
    c.wheelRadiusMeters = wheelRadiusMeters;
    c.wheelWidthMeters = wheelWidthMeters;
    c.wheelInertiaKgMetersSq = wheelInertiaKgMetersSq;
    c.driveMotor = driveMotor;
    c.driveGearRatio = driveGearRatio;
    c.driveEfficiency = driveEfficiency;
    c.driveFrictionTorqueNewtonMeters = driveFrictionTorqueNewtonMeters;
    c.driveStatorCurrentLimitAmps = driveStatorCurrentLimitAmps;
    c.driveSupplyCurrentLimitAmps = driveSupplyCurrentLimitAmps;
    c.driveNeutralMode = driveNeutralMode;
    c.steerMotor = steerMotor;
    c.steerGearRatio = steerGearRatio;
    c.steerEfficiency = steerEfficiency;
    c.steerInertiaKgMetersSq = steerInertiaKgMetersSq;
    c.steerFrictionTorqueNewtonMeters = steerFrictionTorqueNewtonMeters;
    c.steerStatorCurrentLimitAmps = steerStatorCurrentLimitAmps;
    c.steerSupplyCurrentLimitAmps = steerSupplyCurrentLimitAmps;
    c.steerNeutralMode = steerNeutralMode;
    c.couplingGearRatio = couplingGearRatio;
    c.tireStaticFriction = tireStaticFriction;
    c.tireKineticFriction = tireKineticFriction;
    c.tireTransitionSlipSpeedMetersPerSec = tireTransitionSlipSpeedMetersPerSec;
    c.scrubRadiusMeters = scrubRadiusMeters;
    return c;
  }

  /** Packing order must match {@code ModuleParam} in {@code native/src/jni/jni_robots.cpp}. */
  void packInto(float[] out, int offset) {
    Objects.requireNonNull(driveMotor, "driveMotor");
    Objects.requireNonNull(steerMotor, "steerMotor");
    Objects.requireNonNull(driveNeutralMode, "driveNeutralMode");
    Objects.requireNonNull(steerNeutralMode, "steerNeutralMode");
    out[offset] = (float) xMeters;
    out[offset + 1] = (float) yMeters;
    out[offset + 2] = (float) wheelRadiusMeters;
    out[offset + 3] = (float) wheelWidthMeters;
    out[offset + 4] = (float) wheelInertiaKgMetersSq;
    driveMotor.packInto(out, offset + 5);
    out[offset + 12] = (float) driveGearRatio;
    out[offset + 13] = (float) driveEfficiency;
    out[offset + 14] = (float) driveFrictionTorqueNewtonMeters;
    out[offset + 15] = (float) driveStatorCurrentLimitAmps;
    out[offset + 16] = (float) driveSupplyCurrentLimitAmps;
    out[offset + 17] = driveNeutralMode.ordinal();
    steerMotor.packInto(out, offset + 18);
    out[offset + 25] = (float) steerGearRatio;
    out[offset + 26] = (float) steerEfficiency;
    out[offset + 27] = (float) steerInertiaKgMetersSq;
    out[offset + 28] = (float) steerFrictionTorqueNewtonMeters;
    out[offset + 29] = (float) steerStatorCurrentLimitAmps;
    out[offset + 30] = (float) steerSupplyCurrentLimitAmps;
    out[offset + 31] = steerNeutralMode.ordinal();
    out[offset + 32] = (float) couplingGearRatio;
    out[offset + 33] = (float) tireStaticFriction;
    out[offset + 34] = (float) tireKineticFriction;
    out[offset + 35] = (float) tireTransitionSlipSpeedMetersPerSec;
    out[offset + 36] = (float) scrubRadiusMeters;
  }
}
