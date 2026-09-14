package org.frcsim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.frcsim.jni.FrcSimJNI;

/**
 * A simulated swerve robot: a zero-copy view of the native {@code frcsim_swerve_robot_io} block.
 *
 * <p>Voltage setters write shared memory that the next {@link SimWorld#step} reads; getters return
 * values from the latest step. Neither crosses into native code or allocates, so they are safe to
 * call every robot period.
 *
 * <p>Units: meters, radians, seconds, volts, amps, newtons. Module angles are continuous
 * (unwrapped), 0 = robot +X, counterclockwise positive. Drive encoder values are motor-rotor side
 * (wheel rotation times gear ratio), so wheel slip shows up as odometry drift.
 */
public final class SwerveRobot {
  // Mirrors frcsim_swerve_robot_io / frcsim_swerve_module_io in frcsim_c.h; verified by tests.
  static final int ROBOT_SIZE = 608;
  static final int GYRO_YAW = 600;
  static final int X = 0;
  static final int Y = 8;
  static final int Z = 16;
  static final int YAW = 24;
  static final int QX = 32;
  static final int QY = 36;
  static final int QZ = 40;
  static final int QW = 44;
  static final int VX = 48;
  static final int VY = 52;
  static final int VZ = 56;
  static final int WX = 60;
  static final int WY = 64;
  static final int WZ = 68;
  static final int BATTERY_VOLTAGE = 72;
  static final int BATTERY_CURRENT = 76;
  static final int BROWNOUT = 80;
  static final int MODULE_COUNT = 84;
  static final int MODULES = 88;
  static final int MODULE_SIZE = 64;
  static final int DRIVE_VOLTAGE = 0;
  static final int STEER_VOLTAGE = 4;
  static final int DRIVE_ROTOR_POSITION = 8;
  static final int STEER_ANGLE = 16;
  static final int DRIVE_ROTOR_VELOCITY = 24;
  static final int STEER_VELOCITY = 28;
  static final int DRIVE_APPLIED_VOLTAGE = 32;
  static final int DRIVE_STATOR_CURRENT = 36;
  static final int DRIVE_SUPPLY_CURRENT = 40;
  static final int STEER_APPLIED_VOLTAGE = 44;
  static final int STEER_STATOR_CURRENT = 48;
  static final int STEER_SUPPLY_CURRENT = 52;
  static final int NORMAL_FORCE = 56;
  static final int SLIP_SPEED = 60;

  private final SimWorld world;
  private final int index;
  private final ByteBuffer io;
  private final int moduleCount;
  private final double[] steerGearRatios;

  SwerveRobot(SimWorld world, int index, ByteBuffer io, SwerveDriveConfig config) {
    if (io.capacity() != ROBOT_SIZE) {
      throw new FrcSimException(
          "native robot I/O block is " + io.capacity() + " bytes, expected " + ROBOT_SIZE);
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
  public void setDriveVoltage(int module, double volts) {
    io.putFloat(moduleOffset(module) + DRIVE_VOLTAGE, (float) volts);
  }

  /**
   * Sets a module's steer motor voltage, held until changed.
   *
   * @param module module index
   * @param volts commanded voltage (clamped to the battery voltage)
   */
  public void setSteerVoltage(int module, double volts) {
    io.putFloat(moduleOffset(module) + STEER_VOLTAGE, (float) volts);
  }

  /**
   * Sets both motor voltages of a module.
   *
   * @param module module index
   * @param driveVolts drive voltage
   * @param steerVolts steer voltage
   */
  public void setModuleVoltages(int module, double driveVolts, double steerVolts) {
    int offset = moduleOffset(module);
    io.putFloat(offset + DRIVE_VOLTAGE, (float) driveVolts);
    io.putFloat(offset + STEER_VOLTAGE, (float) steerVolts);
  }

  /**
   * Places the robot on the carpet at rest.
   *
   * @param x field x (m)
   * @param y field y (m)
   * @param yawRadians heading
   */
  public void resetPose(double x, double y, double yawRadians) {
    FrcSimJNI.robotResetPose(world.nativeHandle(), index, (float) x, (float) y, (float) yawRadians);
  }

  // ---- Chassis outputs ----------------------------------------------------------------------

  /**
   * Field x of the robot origin (m).
   *
   * @return x
   */
  public double x() {
    return readDouble(X);
  }

  /**
   * Field y of the robot origin (m).
   *
   * @return y
   */
  public double y() {
    return readDouble(Y);
  }

  /**
   * Height of the robot origin (m); about 0 on flat carpet.
   *
   * @return z
   */
  public double z() {
    return readDouble(Z);
  }

  /**
   * True continuous (unwrapped) yaw, counterclockwise positive: ground truth.
   *
   * @return yaw in radians
   */
  public double yawRadians() {
    return readDouble(YAW);
  }

  /**
   * Gyro reading: continuous yaw with the configured scale error, drift, and noise (equal to {@link
   * #yawRadians()} with ideal sensors). Resetting the pose re-zeros the gyro to the new heading.
   *
   * @return measured yaw in radians
   */
  public double gyroYawRadians() {
    return readDouble(GYRO_YAW);
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
   * Center-of-mass velocity along field x (m/s).
   *
   * @return vx
   */
  public double vx() {
    return readFloat(VX);
  }

  /**
   * Center-of-mass velocity along field y (m/s).
   *
   * @return vy
   */
  public double vy() {
    return readFloat(VY);
  }

  /**
   * Center-of-mass velocity along field z (m/s).
   *
   * @return vz
   */
  public double vz() {
    return readFloat(VZ);
  }

  /**
   * Angular velocity about field x (rad/s).
   *
   * @return wx
   */
  public double wx() {
    return readFloat(WX);
  }

  /**
   * Angular velocity about field y (rad/s).
   *
   * @return wy
   */
  public double wy() {
    return readFloat(WY);
  }

  /**
   * Yaw rate, angular velocity about field z (rad/s).
   *
   * @return wz
   */
  public double wz() {
    return readFloat(WZ);
  }

  /**
   * Battery bus voltage after sag (V).
   *
   * @return voltage
   */
  public double batteryVoltage() {
    return readFloat(BATTERY_VOLTAGE);
  }

  /**
   * Total supply current drawn by all simulated motors (A).
   *
   * @return current
   */
  public double batteryCurrent() {
    return readFloat(BATTERY_CURRENT);
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
   * Drive encoder reading: rotor position (rad), i.e. wheel rotation times gear ratio, quantized to
   * {@link SwerveDriveConfig#driveEncoderCountsPerRev} when set.
   *
   * @param module module index
   * @return rotor position
   */
  public double driveRotorPosition(int module) {
    world.checkOpen();
    return io.getDouble(moduleOffset(module) + DRIVE_ROTOR_POSITION);
  }

  /**
   * Drive motor rotor velocity (rad/s).
   *
   * @param module module index
   * @return rotor velocity
   */
  public double driveRotorVelocity(int module) {
    return readModuleFloat(module, DRIVE_ROTOR_VELOCITY);
  }

  /**
   * Continuous module angle (rad), as an absolute encoder on the module would read it (unwrapped).
   *
   * @param module module index
   * @return module angle
   */
  public double steerAngle(int module) {
    world.checkOpen();
    return io.getDouble(moduleOffset(module) + STEER_ANGLE);
  }

  /**
   * Steer motor rotor position (rad), i.e. module angle times steer gear ratio.
   *
   * @param module module index
   * @return rotor position
   */
  public double steerRotorPosition(int module) {
    return steerAngle(module) * steerGearRatios[module];
  }

  /**
   * Module angular velocity about the steer axis (rad/s).
   *
   * @param module module index
   * @return steer velocity
   */
  public double steerVelocity(int module) {
    return readModuleFloat(module, STEER_VELOCITY);
  }

  /**
   * Voltage the drive controller actually applied (lower than commanded under current limiting).
   *
   * @param module module index
   * @return volts
   */
  public double driveAppliedVoltage(int module) {
    return readModuleFloat(module, DRIVE_APPLIED_VOLTAGE);
  }

  /**
   * Drive stator current (A).
   *
   * @param module module index
   * @return amps
   */
  public double driveStatorCurrent(int module) {
    return readModuleFloat(module, DRIVE_STATOR_CURRENT);
  }

  /**
   * Drive supply current (A).
   *
   * @param module module index
   * @return amps
   */
  public double driveSupplyCurrent(int module) {
    return readModuleFloat(module, DRIVE_SUPPLY_CURRENT);
  }

  /**
   * Voltage the steer controller actually applied.
   *
   * @param module module index
   * @return volts
   */
  public double steerAppliedVoltage(int module) {
    return readModuleFloat(module, STEER_APPLIED_VOLTAGE);
  }

  /**
   * Steer stator current (A).
   *
   * @param module module index
   * @return amps
   */
  public double steerStatorCurrent(int module) {
    return readModuleFloat(module, STEER_STATOR_CURRENT);
  }

  /**
   * Steer supply current (A).
   *
   * @param module module index
   * @return amps
   */
  public double steerSupplyCurrent(int module) {
    return readModuleFloat(module, STEER_SUPPLY_CURRENT);
  }

  /**
   * Normal force on the module's wheel (N); shows load transfer.
   *
   * @param module module index
   * @return newtons
   */
  public double normalForce(int module) {
    return readModuleFloat(module, NORMAL_FORCE);
  }

  /**
   * Slip speed of the wheel's contact patch (m/s); near zero while rolling.
   *
   * @param module module index
   * @return m/s
   */
  public double slipSpeed(int module) {
    return readModuleFloat(module, SLIP_SPEED);
  }

  private int moduleOffset(int module) {
    world.checkOpen();
    if (module < 0 || module >= moduleCount) {
      throw new IndexOutOfBoundsException(
          "module " + module + " out of range 0.." + (moduleCount - 1));
    }
    return MODULES + module * MODULE_SIZE;
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
