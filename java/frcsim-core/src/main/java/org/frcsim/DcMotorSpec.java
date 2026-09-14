package org.frcsim;

/**
 * DC motor parameters in WPILib {@code DCMotor} form. {@code count} identical motors drive one
 * gearbox. Presets use WPILib 2026 constants; rotor inertias are estimates (not published by
 * vendors).
 *
 * @param nominalVoltage nominal voltage (V)
 * @param stallTorque stall torque of one motor (N·m)
 * @param stallCurrent stall current of one motor (A)
 * @param freeCurrent free current of one motor (A)
 * @param freeSpeed free speed (rad/s)
 * @param count number of motors
 * @param rotorInertia rotor inertia of one motor (kg·m²)
 */
public record DcMotorSpec(
    double nominalVoltage,
    double stallTorque,
    double stallCurrent,
    double freeCurrent,
    double freeSpeed,
    int count,
    double rotorInertia) {

  private static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;

  private static DcMotorSpec preset(
      double stallTorque,
      double stallCurrent,
      double freeCurrent,
      double freeSpeedRpm,
      int count,
      double rotorInertia) {
    return new DcMotorSpec(
        12.0,
        stallTorque,
        stallCurrent,
        freeCurrent,
        freeSpeedRpm * RPM_TO_RAD_PER_SEC,
        count,
        rotorInertia);
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
   * REV NEO.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec neo(int count) {
    return preset(2.6, 105, 1.8, 5676, count, 4.0e-5);
  }

  /**
   * REV NEO Vortex.
   *
   * @param count number of motors
   * @return spec
   */
  public static DcMotorSpec neoVortex(int count) {
    return preset(3.60, 211, 3.6, 6784, count, 5.0e-5);
  }

  /**
   * Returns a copy with a different rotor inertia.
   *
   * @param value rotor inertia of one motor (kg·m²)
   * @return modified copy
   */
  public DcMotorSpec withRotorInertia(double value) {
    return new DcMotorSpec(
        nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, count, value);
  }

  void packInto(float[] out, int offset) {
    out[offset] = (float) nominalVoltage;
    out[offset + 1] = (float) stallTorque;
    out[offset + 2] = (float) stallCurrent;
    out[offset + 3] = (float) freeCurrent;
    out[offset + 4] = (float) freeSpeed;
    out[offset + 5] = count;
    out[offset + 6] = (float) rotorInertia;
  }
}
