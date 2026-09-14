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

  /** Mass including bumpers and battery (kg). */
  public double mass = 60;

  /** Half bumper-to-bumper length along X (m). */
  public double frameHalfX = 0.45;

  /** Half bumper-to-bumper width along Y (m). */
  public double frameHalfY = 0.45;

  /** Height of the bumper bottom above the carpet (m). */
  public double bumperBottom = 0.02;

  /** Height of the bumper collision box (m). */
  public double bumperHeight = 0.16;

  /** Center of mass X in the robot frame (m). */
  public double comX = 0;

  /** Center of mass Y in the robot frame (m). */
  public double comY = 0;

  /** Center of mass height above the carpet (m). */
  public double comHeight = 0.18;

  /** Yaw moment of inertia (kg·m²); 0 computes it from a uniform box. */
  public double yawInertia = 0;

  /** Bumper material name (see {@link Materials}). */
  public String bumperMaterial = Materials.BUMPER;

  /** Suspension travel standing in for tread and carpet compliance (m). */
  public double suspensionTravel = 0.01;

  /** Suspension natural frequency (Hz). */
  public double suspensionFrequency = 15;

  /** Suspension damping ratio. */
  public double suspensionDampingRatio = 0.7;

  /** Battery open-circuit voltage (V). Estimate. */
  public double batteryOpenCircuitVoltage = 12.5;

  /** Battery plus wiring resistance (Ω). Estimate. */
  public double batteryInternalResistance = 0.02;

  /** Bus voltage below which motor outputs are disabled (V). */
  public double batteryBrownoutVoltage = 6.75;

  /** Bus voltage above which outputs re-enable after a brownout (V). */
  public double batteryBrownoutRecoveryVoltage = 7.5;

  /** Gyro yaw white noise standard deviation (rad); 0 for an ideal gyro. */
  public double gyroYawNoise = 0;

  /** Gyro yaw drift since the last pose reset (rad/s). */
  public double gyroYawDriftRate = 0;

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
   * @param trackWidth distance between left and right module centers (m)
   * @param wheelBase distance between front and back module centers (m)
   * @return config
   */
  public static SwerveDriveConfig rectangular(double trackWidth, double wheelBase) {
    return rectangular(trackWidth, wheelBase, new SwerveModuleConfig());
  }

  /**
   * A robot with four copies of {@code template} at (±wheelBase/2, ±trackWidth/2), ordered
   * front-left, front-right, back-left, back-right.
   *
   * @param trackWidth distance between left and right module centers (m)
   * @param wheelBase distance between front and back module centers (m)
   * @param template module settings to copy
   * @return config
   */
  public static SwerveDriveConfig rectangular(
      double trackWidth, double wheelBase, SwerveModuleConfig template) {
    Objects.requireNonNull(template, "template");
    SwerveDriveConfig config = new SwerveDriveConfig();
    double hx = wheelBase / 2.0;
    double hy = trackWidth / 2.0;
    config.modules.add(template.copy().withPosition(hx, hy));
    config.modules.add(template.copy().withPosition(hx, -hy));
    config.modules.add(template.copy().withPosition(-hx, hy));
    config.modules.add(template.copy().withPosition(-hx, -hy));
    return config;
  }

  /** Packing order must match {@code RobotParam} in {@code native/src/jni/frcsim_jni.cpp}. */
  float[] packRobot() {
    return new float[] {
      (float) mass,
      (float) frameHalfX,
      (float) frameHalfY,
      (float) bumperBottom,
      (float) bumperHeight,
      (float) comX,
      (float) comY,
      (float) comHeight,
      (float) yawInertia,
      (float) suspensionTravel,
      (float) suspensionFrequency,
      (float) suspensionDampingRatio,
      (float) batteryOpenCircuitVoltage,
      (float) batteryInternalResistance,
      (float) batteryBrownoutVoltage,
      (float) batteryBrownoutRecoveryVoltage,
      (float) gyroYawNoise,
      (float) gyroYawDriftRate,
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
