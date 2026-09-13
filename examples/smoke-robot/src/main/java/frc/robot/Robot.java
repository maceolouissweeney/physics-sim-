package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcsim.FrcSim;
import org.frcsim.SimWorld;

/** Smoke-test robot: steps an frcsim world during simulation. */
public class Robot extends TimedRobot {
  private SimWorld world;

  @Override
  public void simulationInit() {
    world = SimWorld.create();
    SmartDashboard.putString("frcsim/version", FrcSim.nativeVersion());
  }

  @Override
  public void simulationPeriodic() {
    world.step(getPeriod());
    SmartDashboard.putNumber("frcsim/timeSeconds", world.timeSeconds());
  }
}
