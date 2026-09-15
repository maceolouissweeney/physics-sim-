package org.frcsim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.frcsim.jni.FrcSimJNI;

/**
 * A simulated swerve robot: a zero-copy view of the native {@code frcsim_swerve_robot_io} block.
 *
 * <p>Command setters write shared memory that the next {@link SimWorld#step} reads; getters return
 * values from the latest step. Neither crosses into native code or allocates, so they are safe to
 * call every robot period.
 *
 * <p>Module angles are continuous (unwrapped), 0 = robot +X, counterclockwise positive. Drive rotor
 * values are motor side (wheel rotation times gear ratio, plus module rotation times the coupling
 * ratio), so wheel slip shows up as odometry drift.
 */
public final class SwerveRobot {
  // Mirrors frcsim_swerve_robot_io / frcsim_swerve_module_io in frcsim_c.h; verified by tests.
  static final int ROBOT_SIZE_BYTES = 608;
  static final int X_METERS = 0;
  static final int Y_METERS = 8;
  static final int Z_METERS = 16;
  static final int YAW_RADIANS = 24;
  static final int QX = 32;
  static final int QY = 36;
  static final int QZ = 40;
  static final int QW = 44;
  static final int VX_METERS_PER_SEC = 48;
  static final int VY_METERS_PER_SEC = 52;
  static final int VZ_METERS_PER_SEC = 56;
  static final int WX_RAD_PER_SEC = 60;
  static final int WY_RAD_PER_SEC = 64;
  static final int WZ_RAD_PER_SEC = 68;
  static final int BATTERY_VOLTS = 72;
  static final int BATTERY_CURRENT_AMPS = 76;
  static final int BROWNOUT = 80;
  static final int MODULE_COUNT = 84;
  static final int MODULES = 88;
  static final int MODULE_SIZE_BYTES = 64;
  static final int GYRO_YAW_RADIANS = 600;
  static final int DRIVE_COMMAND_VOLTS = 0;
  static final int STEER_COMMAND_VOLTS = 4;
  static final int DRIVE_ROTOR_POSITION_RADIANS = 8;
  static final int STEER_ANGLE_RADIANS = 16;
  static final int DRIVE_ROTOR_VELOCITY_RAD_PER_SEC = 24;
  static final int STEER_VELOCITY_RAD_PER_SEC = 28;
  static final int DRIVE_APPLIED_VOLTS = 32;
  static final int DRIVE_STATOR_CURRENT_AMPS = 36;
  static final int DRIVE_SUPPLY_CURRENT_AMPS = 40;
  static final int STEER_APPLIED_VOLTS = 44;
  static final int STEER_STATOR_CURRENT_AMPS = 48;
  static final int STEER_SUPPLY_CURRENT_AMPS = 52;
  static final int NORMAL_FORCE_NEWTONS = 56;
  static final int SLIP_SPEED_METERS_PER_SEC = 60;

  private final SimWorld world;
  private final int index;
  private final ByteBuffer io;
  private final int moduleCount;
  private final double[] steerGearRatios;

  SwerveRobot(SimWorld world, int index, ByteBuffer io, SwerveDriveConfig config) {
    if (io.capacity() != ROBOT_SIZE_BYTES) {
      throw new FrcSimException(
          "native robot I/O block is " + io.capacity() + " bytes, expected " + ROBOT_SIZE_BYTES);
    }
    this.world = world;
    this.index = index;
    this.io = io.order(ByteOrder.nativeOrder());
    this.moduleCount = this.io.getInt(MODULE_COUNT);
    this.steerGearRatios = new double[moduleCount];
    for (int m = 0; m < moduleCount; m++) {
      steerGearRatios[m] = config.modules.get(m).steerGearRatio;
    }
  }

  /**
   * Robot index within its world.
   *
   * @return index
   */
  public int index() {
    return index;
  }

  /**
   * Number of swerve modules.
   *
   * @return module count
   */
  public int moduleCount() {
    return moduleCount;
  }

  // ---- Inputs -------------------------------------------------------------------------------

  /**
   * Sets a module's drive motor voltage, held until changed.
   *
   * @param module module index
   * @param volts commanded voltage (clamped to the battery voltage)
   */
  public void setDriveCommandVolts(int module, double volts) {
    io.putFloat(moduleOffset(module) + DRIVE_COMMAND_VOLTS, (float) volts);
  }

  /**
   * Sets a module's steer motor voltage, held until changed.
   *
   * @param module module index
   * @param volts commanded voltage (clamped to the battery voltage)
   */
  public void setSteerCommandVolts(int module, double volts) {
    io.putFloat(moduleOffset(module) + STEER_COMMAND_VOLTS, (float) volts);
  }

  /**
   * Sets both motor voltages of a module.
   *
   * @param module module index
   * @param driveVolts drive voltage
   * @param steerVolts steer voltage
   */
  public void setModuleCommandVolts(int module, double driveVolts, double steerVolts) {
    int offset = moduleOffset(module);
    io.putFloat(offset + DRIVE_COMMAND_VOLTS, (float) driveVolts);
    io.putFloat(offset + STEER_COMMAND_VOLTS, (float) steerVolts);
  }

  /**
   * Places the robot on the carpet at rest and re-zeros the gyro to the new heading.
   *
   * @param xMeters field x
   * @param yMeters field y
   * @param yawRadians heading
   */
  public void resetPose(double xMeters, double yMeters, double yawRadians) {
    FrcSimJNI.robotResetPose(
        world.nativeHandle(), index, (float) xMeters, (float) yMeters, (float) yawRadians);
  }

  // ---- Chassis outputs ----------------------------------------------------------------------

  /**
   * Field x of the robot origin.
   *
   * @return meters
   */
  public double xMeters() {
    return readDouble(X_METERS);
  }

  /**
   * Field y of the robot origin.
   *
   * @return meters
   */
  public double yMeters() {
    return readDouble(Y_METERS);
  }

  /**
   * Height of the robot origin; about 0 on flat carpet.
   *
   * @return meters
   */
  public double zMeters() {
    return readDouble(Z_METERS);
  }

  /**
   * True continuous (unwrapped) yaw, counterclockwise positive: ground truth.
   *
   * @return radians
   */
  public double yawRadians() {
    return readDouble(YAW_RADIANS);
  }

  /**
   * Gyro reading: continuous yaw with the configured scale error, drift, and noise (equal to {@link
   * #yawRadians()} with ideal sensors). Resetting the pose re-zeros the gyro to the new heading.
   *
   * @return radians
   */
  public double gyroYawRadians() {
    return readDouble(GYRO_YAW_RADIANS);
  }

  /**
   * Orientation quaternion x.
   *
   * @return qx
   */
  public double qx() {
    return readFloat(QX);
  }

  /**
   * Orientation quaternion y.
   *
   * @return qy
   */
  public double qy() {
    return readFloat(QY);
  }

  /**
   * Orientation quaternion z.
   *
   * @return qz
   */
  public double qz() {
    return readFloat(QZ);
  }

  /**
   * Orientation quaternion w.
   *
   * @return qw
   */
  public double qw() {
    return readFloat(QW);
  }

  /**
   * Center-of-mass velocity along field x.
   *
   * @return m/s
   */
  public double vxMetersPerSec() {
    return readFloat(VX_METERS_PER_SEC);
  }

  /**
   * Center-of-mass velocity along field y.
   *
   * @return m/s
   */
  public double vyMetersPerSec() {
    return readFloat(VY_METERS_PER_SEC);
  }

  /**
   * Center-of-mass velocity along field z.
   *
   * @return m/s
   */
  public double vzMetersPerSec() {
    return readFloat(VZ_METERS_PER_SEC);
  }

  /**
   * Angular velocity about field x.
   *
   * @return rad/s
   */
  public double wxRadPerSec() {
    return readFloat(WX_RAD_PER_SEC);
  }

  /**
   * Angular velocity about field y.
   *
   * @return rad/s
   */
  public double wyRadPerSec() {
    return readFloat(WY_RAD_PER_SEC);
  }

  /**
   * Yaw rate: angular velocity about field z.
   *
   * @return rad/s
   */
  public double wzRadPerSec() {
    return readFloat(WZ_RAD_PER_SEC);
  }

  /**
   * Battery bus voltage after sag.
   *
   * @return volts
   */
  public double batteryVolts() {
    return readFloat(BATTERY_VOLTS);
  }

  /**
   * Total supply current drawn by all simulated motors.
   *
   * @return amps
   */
  public double batteryCurrentAmps() {
    return readFloat(BATTERY_CURRENT_AMPS);
  }

  /**
   * Whether motor outputs are disabled by a brownout.
   *
   * @return true during brownout
   */
  public boolean isBrownedOut() {
    world.checkOpen();
    return io.getInt(BROWNOUT) != 0;
  }

  // ---- Module outputs -----------------------------------------------------------------------

  /**
   * Drive encoder reading: rotor position, i.e. wheel rotation times gear ratio plus module
   * rotation times {@link SwerveModuleConfig#couplingGearRatio}, quantized to {@link
   * SwerveDriveConfig#driveEncoderCountsPerRev} when set.
   *
   * @param module module index
   * @return radians
   */
  public double driveRotorPositionRadians(int module) {
    return io.getDouble(moduleOffset(module) + DRIVE_ROTOR_POSITION_RADIANS);
  }

  /**
   * Drive motor rotor velocity, including the coupling term.
   *
   * @param module module index
   * @return rad/s
   */
  public double driveRotorVelocityRadPerSec(int module) {
    return readModuleFloat(module, DRIVE_ROTOR_VELOCITY_RAD_PER_SEC);
  }

  /**
   * Continuous module angle, as an absolute encoder on the module would read it (unwrapped).
   *
   * @param module module index
   * @return radians
   */
  public double steerAngleRadians(int module) {
    return io.getDouble(moduleOffset(module) + STEER_ANGLE_RADIANS);
  }

  /**
   * Module angular velocity about the steer axis.
   *
   * @param module module index
   * @return rad/s
   */
  public double steerVelocityRadPerSec(int module) {
    return readModuleFloat(module, STEER_VELOCITY_RAD_PER_SEC);
  }

  /**
   * Steer motor rotor position: module angle times steer gear ratio.
   *
   * @param module module index
   * @return radians
   */
  public double steerRotorPositionRadians(int module) {
    return steerAngleRadians(module) * steerGearRatios[module];
  }

  /**
   * Steer motor rotor velocity: module angular velocity times steer gear ratio.
   *
   * @param module module index
   * @return rad/s
   */
  public double steerRotorVelocityRadPerSec(int module) {
    return steerVelocityRadPerSec(module) * steerGearRatios[module];
  }

  /**
   * Voltage the drive controller actually applied (lower than commanded under current limiting).
   *
   * @param module module index
   * @return volts
   */
  public double driveAppliedVolts(int module) {
    return readModuleFloat(module, DRIVE_APPLIED_VOLTS);
  }

  /**
   * Drive stator current.
   *
   * @param module module index
   * @return amps
   */
  public double driveStatorCurrentAmps(int module) {
    return readModuleFloat(module, DRIVE_STATOR_CURRENT_AMPS);
  }

  /**
   * Drive supply current.
   *
   * @param module module index
   * @return amps
   */
  public double driveSupplyCurrentAmps(int module) {
    return readModuleFloat(module, DRIVE_SUPPLY_CURRENT_AMPS);
  }

  /**
   * Voltage the steer controller actually applied.
   *
   * @param module module index
   * @return volts
   */
  public double steerAppliedVolts(int module) {
    return readModuleFloat(module, STEER_APPLIED_VOLTS);
  }

  /**
   * Steer stator current.
   *
   * @param module module index
   * @return amps
   */
  public double steerStatorCurrentAmps(int module) {
    return readModuleFloat(module, STEER_STATOR_CURRENT_AMPS);
  }

  /**
   * Steer supply current.
   *
   * @param module module index
   * @return amps
   */
  public double steerSupplyCurrentAmps(int module) {
    return readModuleFloat(module, STEER_SUPPLY_CURRENT_AMPS);
  }

  /**
   * Normal force on the module's wheel; shows load transfer.
   *
   * @param module module index
   * @return newtons
   */
  public double normalForceNewtons(int module) {
    return readModuleFloat(module, NORMAL_FORCE_NEWTONS);
  }

  /**
   * Slip speed of the wheel's contact patch; near zero while rolling.
   *
   * @param module module index
   * @return m/s
   */
  public double slipSpeedMetersPerSec(int module) {
    return readModuleFloat(module, SLIP_SPEED_METERS_PER_SEC);
  }

  private int moduleOffset(int module) {
    world.checkOpen();
    if (module < 0 || module >= moduleCount) {
      throw new IndexOutOfBoundsException(
          "module " + module + " out of range 0.." + (moduleCount - 1));
    }
    return MODULES + module * MODULE_SIZE_BYTES;
  }

  private double readDouble(int offset) {
    world.checkOpen();
    return io.getDouble(offset);
  }

  private double readFloat(int offset) {
    world.checkOpen();
    return io.getFloat(offset);
  }

  private double readModuleFloat(int module, int field) {
    return io.getFloat(moduleOffset(module) + field);
  }
}
