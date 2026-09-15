# Swerve quickstart

Simulate a swerve robot in a WPILib Java project. Model background: [models/swerve.md](../models/swerve.md).
frcsim targets CTRE Phoenix 6 hardware; REV motor controllers are not supported.

> Status: the frcsim core and `frcsim-wpilib` APIs below are tested. The CTRE snippet is a manual recipe; the
> `frcsim-ctre` module (IMPLEMENTATION_PLAN P2.14) will do this wiring for a Phoenix 6 `SwerveDrivetrain`.

Every quantity with a unit carries it in its name (`wheelRadiusMeters`, `driveRotorPositionRadians`, ...).

## 1. Create the world and robot

```java
import org.frcsim.*;
import org.frcsim.wpilib.*;

public class DriveSim {
  private final SimWorld world;
  private final SimSwerveDrive drive;

  public DriveSim(Pose2d startPose) {
    SimulationGuard.requireSimulation();
    world = SimWorld.create();
    world.field().loadJson(Filesystem.getDeployDirectory().toPath().resolve("fields/test-flat/field.json"));

    // Describe your drivetrain (Phoenix-style config objects: set the fields you know).
    SwerveModuleConfig module = new SwerveModuleConfig();
    module.driveMotor = DcMotorSpec.krakenX60Foc(1);
    module.driveGearRatio = 6.12;                          // L3
    module.couplingGearRatio = 50.0 / 14.0;                // TunerConstants kCoupleRatio
    module.wheelRadiusMeters = Units.inchesToMeters(2);
    module.tireStaticFriction = 1.2;                       // measure with a pull test if you can

    SwerveDriveConfig config = SwerveDriveConfig.rectangular(0.57, 0.57, module);
    config.massKg = 58;
    config.comHeightMeters = 0.20;
    drive = SimSwerveDrive.create(world, config, startPose);
  }
}
```

## 2. Every simulation period

The motor controllers own the voltages. Take their applied voltages, step the world once, then write the
simulated sensor values back.

```java
@Override
public void simulationPeriodic() {
  for (int m = 0; m < 4; m++) {
    drive.setModuleCommandVolts(m, driveMotorVolts(m), steerMotorVolts(m)); // from TalonFX sim states
  }
  world.step(0.020);                                                        // 5 physics substeps

  RoboRioSim.setVInVoltage(drive.getBatteryVolts());                        // battery sag for the whole robot
  // ...feed encoders (below), gyro, and telemetry
}
```

### CTRE Phoenix 6 (TalonFX + CANcoder + Pigeon 2)
```java
SwerveRobot robot = drive.getRobot();
TalonFXSimState driveSim = driveMotor.getSimState();
driveSim.Orientation = driveInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
driveSim.setSupplyVoltage(drive.getBatteryVolts());
double driveVolts = driveSim.getMotorVoltage();                           // input to frcsim
// after world.step():
driveSim.setRawRotorPosition(Units.radiansToRotations(robot.driveRotorPositionRadians(m)));
driveSim.setRotorVelocity(Units.radiansToRotations(robot.driveRotorVelocityRadPerSec(m)));
steerSim.setRawRotorPosition(Units.radiansToRotations(robot.steerRotorPositionRadians(m)));
steerSim.setRotorVelocity(Units.radiansToRotations(robot.steerRotorVelocityRadPerSec(m)));
cancoder.getSimState().setRawPosition(Units.radiansToRotations(robot.steerAngleRadians(m)));
cancoder.getSimState().setVelocity(Units.radiansToRotations(robot.steerVelocityRadPerSec(m)));
pigeon.getSimState().setRawYaw(Math.toDegrees(drive.getGyroYawRadians()));
pigeon.getSimState().setAngularVelocityZ(Math.toDegrees(drive.getGyroYawRateRadPerSec()));
```
frcsim's positive drive voltage rolls the wheel forward and positive steer voltage turns the module
counterclockwise; the sim state `Orientation` fields map those to your motor and encoder inversions.

### Plain WPILib (no vendor sim)
Run your own PID on the sim encoders and call `setModuleCommandVolts` directly. For stiff steering, step
the world in smaller increments (e.g. `world.step(0.004, 1)` five times per period) so your loop runs at
250 Hz like a motor controller's onboard loop.

## 3. Odometry vs ground truth

- `drive.getModulePositions()` and `drive.getGyroYaw()` are **sensor values**: wheel slip shows up as drift,
  so feed them to your `SwerveDrivePoseEstimator` just like on the real robot.
- `drive.getPose()` / `getPose3d()` is **ground truth**: log it next to your estimate in AdvantageScope to see
  estimator error.
