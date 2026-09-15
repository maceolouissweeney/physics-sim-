package org.frcsim;

/**
 * DC motor parameters in WPILib {@code DCMotor} form. {@code count} identical motors drive one
 * gearbox. Presets are CTRE motors with WPILib 2026 constants; rotor inertias are estimates (not
 * published by vendors).
 *
 * @param nominalVolts nominal voltage
 * @param stallTorqueNewtonMeters stall torque of one motor
 * @param stallCurrentAmps stall current of one motor
 * @param freeCurrentAmps free current of one motor
 * @param freeSpeedRadPerSec free speed
 * @param count number of motors
 * @param rotorInertiaKgMetersSq rotor inertia of one motor
 */
public record DcMotorSpec(
    double nominalVolts,
    double stallTorqueNewtonMeters,
    double stallCurrentAmps,
    double freeCurrentAmps,
    double freeSpeedRadPerSec,
    int count,
    double rotorInertiaKgMetersSq) {

  private static final double RAD_PER_SEC_PER_RPM = 2.0 * Math.PI / 60.0;

  private static DcMotorSpec preset(
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRpm,
      int count,
      double rotorInertiaKgMetersSq) {
    return new DcMotorSpec(
        12.0,
        stallTorqueNewtonMeters,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRpm * RAD_PER_SEC_PER_RPM,
        count,
        rotorInertiaKgMetersSq);
  }

  /**
   * Kraken X60 (trapezoidal commutation).
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec krakenX60(int count) {
    return preset(7.09, 366, 2, 6000, count, 6.0e-5);
  }

  /**
   * Kraken X60 with FOC.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec krakenX60Foc(int count) {
    return preset(9.37, 483, 2, 5800, count, 6.0e-5);
  }

  /**
   * Kraken X44.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec krakenX44(int count) {
    return preset(4.11, 279, 2, 7758, count, 3.0e-5);
  }

  /**
   * Kraken X44 with FOC.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec krakenX44Foc(int count) {
    return preset(5.01, 329, 2, 7368, count, 3.0e-5);
  }

  /**
   * Falcon 500.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec falcon500(int count) {
    return preset(4.69, 257, 1.5, 6380, count, 5.5e-5);
  }

  /**
   * Falcon 500 with FOC.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec falcon500Foc(int count) {
    return preset(5.84, 304, 1.5, 6080, count, 5.5e-5);
  }

  /**
   * CTRE Minion (driven by a Talon FXS).
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec minion(int count) {
    return preset(3.17, 211, 2, 7704, count, 3.5e-5);
  }

  /**
   * Returns a copy with a different rotor inertia.
   *
   * @param value rotor inertia of one motor (kg·m²)
   * @return modified copy
   */
  public DcMotorSpec withRotorInertiaKgMetersSq(double value) {
    return new DcMotorSpec(
        nominalVolts,
        stallTorqueNewtonMeters,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec,
        count,
        value);
  }

  /**
   * Returns a copy driving a different number of motors.
   *
   * @param value number of motors
   * @return modified copy
   */
  public DcMotorSpec withCount(int value) {
    return new DcMotorSpec(
        nominalVolts,
        stallTorqueNewtonMeters,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec,
        value,
        rotorInertiaKgMetersSq);
  }

  /** Values in the packed JNI representation. */
  static final int PACKED_SIZE = 7;

  void packInto(float[] out, int offset) {
    out[offset] = (float) nominalVolts;
    out[offset + 1] = (float) stallTorqueNewtonMeters;
    out[offset + 2] = (float) stallCurrentAmps;
    out[offset + 3] = (float) freeCurrentAmps;
    out[offset + 4] = (float) freeSpeedRadPerSec;
    out[offset + 5] = count;
    out[offset + 6] = (float) rotorInertiaKgMetersSq;
  }
}
