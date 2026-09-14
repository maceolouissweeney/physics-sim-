package org.frcsim.wpilib;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Objects;
import org.frcsim.DcMotorSpec;

/** Conversions from WPILib motor models. */
public final class WpilibMotors {
  private WpilibMotors() {}

  /**
   * Converts a WPILib {@link DCMotor}. WPILib already folds the motor count into stall torque and
   * currents, so the result describes all motors as one ({@code count = 1}) with identical R, Kv,
   * and Kt.
   *
   * @param motor WPILib motor model, e.g. {@code DCMotor.getKrakenX60Foc(1)}
   * @param totalRotorInertia combined rotor inertia of all motors in the model (kg·m²)
   * @return frcsim motor spec
   */
  public static DcMotorSpec fromDCMotor(DCMotor motor, double totalRotorInertia) {
    Objects.requireNonNull(motor, "motor");
    return new DcMotorSpec(
        motor.nominalVoltageVolts,
        motor.stallTorqueNewtonMeters,
        motor.stallCurrentAmps,
        motor.freeCurrentAmps,
        motor.freeSpeedRadPerSec,
        1,
        totalRotorInertia);
  }
}
