package org.frcsim;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Configuration of a swerve robot, in the style of CTRE Phoenix 6 configs: public fields with
 * defaults, validated when the robot is added with {@link Robots#addSwerve}. Robot frame: origin on
 * the carpet under the frame center, +X forward, +Y left, +Z up.
 */
public final class SwerveDriveConfig {
  /** Maximum modules per robot. */
  public static final int MAX_MODULES = 8;

  /** Mass including bumpers and battery. */
  public double massKg = 60;

  /** Half bumper-to-bumper length along X. */
  public double frameHalfXMeters = 0.45;

  /** Half bumper-to-bumper width along Y. */
  public double frameHalfYMeters = 0.45;

  /** Height of the bumper bottom above the carpet. */
  public double bumperBottomMeters = 0.02;

  /** Height of the bumper collision box. */
  public double bumperHeightMeters = 0.16;

  /** Center of mass X in the robot frame. */
  public double comXMeters = 0;

  /** Center of mass Y in the robot frame. */
  public double comYMeters = 0;

  /** Center of mass height above the carpet. */
  public double comHeightMeters = 0.18;

  /** Yaw moment of inertia; 0 computes it from a uniform box. */
  public double yawInertiaKgMetersSq = 0;

  /** Bumper material name (see {@link Materials}). */
  public String bumperMaterial = Materials.BUMPER;

  /** Suspension travel standing in for tread and carpet compliance. */
  public double suspensionTravelMeters = 0.01;

  /** Suspension natural frequency. */
  public double suspensionFrequencyHz = 15;

  /** Suspension damping ratio. */
  public double suspensionDampingRatio = 0.7;

  /** Battery open-circuit voltage. Estimate. */
  public double batteryOpenCircuitVolts = 12.5;

  /** Battery plus wiring resistance. Estimate. */
  public double batteryInternalResistanceOhms = 0.02;

  /** Bus voltage below which motor outputs are disabled. */
  public double batteryBrownoutVolts = 6.75;

  /** Bus voltage above which outputs re-enable after a brownout. */
  public double batteryBrownoutRecoveryVolts = 7.5;

  /** Gyro yaw white noise standard deviation; 0 for an ideal gyro. */
  public double gyroYawNoiseRadians = 0;

  /** Gyro yaw drift since the last pose reset. */
  public double gyroYawDriftRateRadPerSec = 0;

  /** Gyro scale error as a fraction, e.g. 0.005 reads 0.5% too much rotation. */
  public double gyroScaleError = 0;

  /** Drive encoder counts per rotor revolution; 0 for continuous readings. */
  public int driveEncoderCountsPerRev = 0;

  /** Seed for sensor noise (deterministic per platform), 0..16777215. */
  public int sensorSeed = 1;

  /** Modules, typically front-left, front-right, back-left, back-right. */
  public final List<SwerveModuleConfig> modules = new ArrayList<>();

  /** Values in the packed JNI representation of the robot fields. */
  static final int PACKED_SIZE = 21;

  /**
   * A robot with four default modules at (±wheelBase/2, ±trackWidth/2), ordered front-left,
   * front-right, back-left, back-right.
   *
   * @param trackWidthMeters distance between left and right module centers
   * @param wheelBaseMeters distance between front and back module centers
   * @return config
   */
  public static SwerveDriveConfig rectangular(double trackWidthMeters, double wheelBaseMeters) {
    return rectangular(trackWidthMeters, wheelBaseMeters, new SwerveModuleConfig());
  }

  /**
   * A robot with four copies of {@code template} at (±wheelBase/2, ±trackWidth/2), ordered
   * front-left, front-right, back-left, back-right.
   *
   * @param trackWidthMeters distance between left and right module centers
   * @param wheelBaseMeters distance between front and back module centers
   * @param template module settings to copy
   * @return config
   */
  public static SwerveDriveConfig rectangular(
      double trackWidthMeters, double wheelBaseMeters, SwerveModuleConfig template) {
    Objects.requireNonNull(template, "template");
    SwerveDriveConfig config = new SwerveDriveConfig();
    double halfWheelBaseMeters = wheelBaseMeters / 2.0;
    double halfTrackWidthMeters = trackWidthMeters / 2.0;
    config.modules.add(template.copy().withPosition(halfWheelBaseMeters, halfTrackWidthMeters));
    config.modules.add(template.copy().withPosition(halfWheelBaseMeters, -halfTrackWidthMeters));
    config.modules.add(template.copy().withPosition(-halfWheelBaseMeters, halfTrackWidthMeters));
    config.modules.add(template.copy().withPosition(-halfWheelBaseMeters, -halfTrackWidthMeters));
    return config;
  }

  /** Packing order must match {@code RobotParam} in {@code native/src/jni/jni_robots.cpp}. */
  float[] packRobot() {
    return new float[] {
      (float) massKg,
      (float) frameHalfXMeters,
      (float) frameHalfYMeters,
      (float) bumperBottomMeters,
      (float) bumperHeightMeters,
      (float) comXMeters,
      (float) comYMeters,
      (float) comHeightMeters,
      (float) yawInertiaKgMetersSq,
      (float) suspensionTravelMeters,
      (float) suspensionFrequencyHz,
      (float) suspensionDampingRatio,
      (float) batteryOpenCircuitVolts,
      (float) batteryInternalResistanceOhms,
      (float) batteryBrownoutVolts,
      (float) batteryBrownoutRecoveryVolts,
      (float) gyroYawNoiseRadians,
      (float) gyroYawDriftRateRadPerSec,
      (float) gyroScaleError,
      driveEncoderCountsPerRev,
      sensorSeed,
    };
  }

  float[] packModules() {
    if (modules.isEmpty() || modules.size() > MAX_MODULES) {
      throw new IllegalArgumentException("a swerve robot needs 1.." + MAX_MODULES + " modules");
    }
    float[] packed = new float[modules.size() * SwerveModuleConfig.PACKED_SIZE];
    for (int i = 0; i < modules.size(); i++) {
      Objects.requireNonNull(modules.get(i), "modules[" + i + "]")
          .packInto(packed, i * SwerveModuleConfig.PACKED_SIZE);
    }
    return packed;
  }
}
