package org.frcsim.wpilib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;
import org.frcsim.SimWorld;
import org.frcsim.SwerveDriveConfig;
import org.frcsim.SwerveModuleConfig;
import org.frcsim.SwerveRobot;

/**
 * WPILib-typed view of a simulated swerve robot: poses, chassis speeds, module states and
 * positions, and gyro angles, derived from the robot's native sensor outputs.
 *
 * <p>Module positions come from the simulated drive encoders, so they include wheel slip, exactly
 * like real odometry. The "true" pose ({@link #getPose()}) is ground truth for comparison.
 */
public final class SimSwerveDrive {
  private final SwerveRobot robot;
  private final double[] driveGearRatios;
  private final double[] couplingGearRatios;
  private final double[] wheelRadiiMeters;
  private final Translation2d[] moduleTranslations;

  /**
   * Wraps an existing robot.
   *
   * @param robot simulated robot
   * @param config the configuration the robot was created with
   */
  public SimSwerveDrive(SwerveRobot robot, SwerveDriveConfig config) {
    this.robot = Objects.requireNonNull(robot, "robot");
    Objects.requireNonNull(config, "config");
    int n = robot.moduleCount();
    if (config.modules.size() != n) {
      throw new IllegalArgumentException("config has a different module count than the robot");
    }
    driveGearRatios = new double[n];
    couplingGearRatios = new double[n];
    wheelRadiiMeters = new double[n];
    moduleTranslations = new Translation2d[n];
    for (int m = 0; m < n; m++) {
      SwerveModuleConfig module = config.modules.get(m);
      driveGearRatios[m] = module.driveGearRatio;
      couplingGearRatios[m] = module.couplingGearRatio;
      wheelRadiiMeters[m] = module.wheelRadiusMeters;
      moduleTranslations[m] = new Translation2d(module.xMeters, module.yMeters);
    }
  }

  /**
   * Adds a swerve robot to a world at a starting pose.
   *
   * @param world simulation world
   * @param config robot configuration
   * @param startPose starting pose on the field
   * @return drive view
   */
  public static SimSwerveDrive create(SimWorld world, SwerveDriveConfig config, Pose2d startPose) {
    Objects.requireNonNull(startPose, "startPose");
    SwerveRobot robot =
        world
            .robots()
            .addSwerve(
                config, startPose.getX(), startPose.getY(), startPose.getRotation().getRadians());
    return new SimSwerveDrive(robot, config);
  }

  /**
   * The underlying robot handle.
   *
   * @return robot
   */
  public SwerveRobot getRobot() {
    return robot;
  }

  /**
   * Module locations relative to the robot center, for {@code SwerveDriveKinematics}.
   *
   * @return module translations in module order
   */
  public Translation2d[] getModuleTranslations() {
    return moduleTranslations.clone();
  }

  // ---- Commands -----------------------------------------------------------------------------

  /**
   * Sets a module's motor voltages (held until changed).
   *
   * @param module module index
   * @param driveVolts drive motor voltage
   * @param steerVolts steer motor voltage
   */
  public void setModuleCommandVolts(int module, double driveVolts, double steerVolts) {
    robot.setModuleCommandVolts(module, driveVolts, steerVolts);
  }

  /**
   * Sets a module's motor voltages (held until changed).
   *
   * @param module module index
   * @param drive drive motor voltage
   * @param steer steer motor voltage
   */
  public void setModuleVoltages(int module, Voltage drive, Voltage steer) {
    robot.setModuleCommandVolts(module, drive.in(Units.Volts), steer.in(Units.Volts));
  }

  /**
   * Teleports the robot to a pose, at rest, and re-zeros the gyro to its heading.
   *
   * @param pose new pose
   */
  public void resetPose(Pose2d pose) {
    robot.resetPose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  // ---- Ground truth -------------------------------------------------------------------------

  /**
   * True robot pose on the field (ground truth, not odometry).
   *
   * @return pose
   */
  public Pose2d getPose() {
    return new Pose2d(robot.xMeters(), robot.yMeters(), new Rotation2d(robot.yawRadians()));
  }

  /**
   * True 3D robot pose (includes pitch and roll on bumps or when tipping).
   *
   * @return pose
   */
  public Pose3d getPose3d() {
    return new Pose3d(
        new Translation3d(robot.xMeters(), robot.yMeters(), robot.zMeters()),
        new Rotation3d(new Quaternion(robot.qw(), robot.qx(), robot.qy(), robot.qz())));
  }

  /**
   * Robot-relative chassis speeds of the center of mass.
   *
   * @return speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    double yawRadians = robot.yawRadians();
    double cos = Math.cos(yawRadians);
    double sin = Math.sin(yawRadians);
    double vxMetersPerSec = robot.vxMetersPerSec();
    double vyMetersPerSec = robot.vyMetersPerSec();
    return new ChassisSpeeds(
        cos * vxMetersPerSec + sin * vyMetersPerSec,
        -sin * vxMetersPerSec + cos * vyMetersPerSec,
        robot.wzRadPerSec());
  }

  /**
   * Field-relative chassis speeds of the center of mass.
   *
   * @return speeds
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return new ChassisSpeeds(robot.vxMetersPerSec(), robot.vyMetersPerSec(), robot.wzRadPerSec());
  }

  // ---- Sensors ------------------------------------------------------------------------------

  /**
   * Gyro yaw (continuous, counterclockwise positive) as a {@link Rotation2d}.
   *
   * @return yaw
   */
  public Rotation2d getGyroYaw() {
    return new Rotation2d(robot.gyroYawRadians());
  }

  /**
   * Continuous gyro yaw, like a Pigeon 2's accumulated yaw.
   *
   * @return radians
   */
  public double getGyroYawRadians() {
    return robot.gyroYawRadians();
  }

  /**
   * Gyro yaw rate, counterclockwise positive.
   *
   * @return rad/s
   */
  public double getGyroYawRateRadPerSec() {
    return robot.wzRadPerSec();
  }

  /**
   * Robot pitch and roll (and yaw) as a {@link Rotation3d}.
   *
   * @return orientation
   */
  public Rotation3d getGyroRotation3d() {
    return new Rotation3d(new Quaternion(robot.qw(), robot.qx(), robot.qy(), robot.qz()));
  }

  /**
   * Module states from the simulated encoders.
   *
   * @return new array of states in module order
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[robot.moduleCount()];
    for (int m = 0; m < states.length; m++) {
      states[m] = new SwerveModuleState();
    }
    updateModuleStates(states);
    return states;
  }

  /**
   * Fills existing module state objects. The wheel speed removes the steering coupling term, like
   * CTRE's odometry does. Each angle is a new Rotation2d (WPILib rotations are immutable).
   *
   * @param states one state per module
   */
  public void updateModuleStates(SwerveModuleState[] states) {
    for (int m = 0; m < robot.moduleCount(); m++) {
      double wheelRadPerSec =
          (robot.driveRotorVelocityRadPerSec(m)
                  - robot.steerVelocityRadPerSec(m) * couplingGearRatios[m])
              / driveGearRatios[m];
      states[m].speedMetersPerSecond = wheelRadPerSec * wheelRadiiMeters[m];
      states[m].angle = new Rotation2d(robot.steerAngleRadians(m));
    }
  }

  /**
   * Module positions from the simulated drive encoders (includes wheel slip).
   *
   * @return new array of positions in module order
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[robot.moduleCount()];
    for (int m = 0; m < positions.length; m++) {
      positions[m] = new SwerveModulePosition();
    }
    updateModulePositions(positions);
    return positions;
  }

  /**
   * Fills existing module position objects. The distance removes the steering coupling term, like
   * CTRE's odometry does. Each angle is a new Rotation2d (WPILib rotations are immutable).
   *
   * @param positions one position per module
   */
  public void updateModulePositions(SwerveModulePosition[] positions) {
    for (int m = 0; m < robot.moduleCount(); m++) {
      double steerAngleRadians = robot.steerAngleRadians(m);
      double wheelRadians =
          (robot.driveRotorPositionRadians(m) - steerAngleRadians * couplingGearRatios[m])
              / driveGearRatios[m];
      positions[m].distanceMeters = wheelRadians * wheelRadiiMeters[m];
      positions[m].angle = new Rotation2d(steerAngleRadians);
    }
  }

  /**
   * Battery bus voltage after sag, for {@code RoboRioSim.setVInVoltage}.
   *
   * @return volts
   */
  public double getBatteryVolts() {
    return robot.batteryVolts();
  }
}
