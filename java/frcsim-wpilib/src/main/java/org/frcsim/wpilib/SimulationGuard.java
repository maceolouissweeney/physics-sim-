package org.frcsim.wpilib;

import edu.wpi.first.wpilibj.RobotBase;

/** Prevents the simulator from being started on real robot hardware. */
public final class SimulationGuard {
  private SimulationGuard() {}

  /**
   * Throws if the robot program is running on a real robot. Call before creating a {@code
   * SimWorld}, e.g. in {@code simulationInit()}.
   *
   * @throws IllegalStateException on a real robot
   */
  public static void requireSimulation() {
    if (RobotBase.isReal()) {
      throw new IllegalStateException(
          "frcsim runs only in desktop simulation; do not create a SimWorld on a real robot");
    }
  }
}
