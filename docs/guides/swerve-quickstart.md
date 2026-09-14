# Swerve quickstart

Simulate a swerve robot in a WPILib Java project. Model background: [models/swerve.md](../models/swerve.md).

> Status: the frcsim core and `frcsim-wpilib` APIs below are tested. The CTRE and REV snippets are
> **recipes, not yet compiled against the vendor libraries** (tracked in IMPLEMENTATION_PLAN P5.6).

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
    module.driveMotor = WpilibMotors.fromDCMotor(DCMotor.getKrakenX60Foc(1), 6.0e-5);
    module.driveGearRatio = 6.12;          // L3
    module.wheelRadius = Units.inchesToMeters(2);
    module.tireStaticFriction = 1.2;       // measure with a pull test if you can

    SwerveDriveConfig config = SwerveDriveConfig.rectangular(0.57, 0.57, module);
    config.mass = 58;
    config.comHeight = 0.20;
    drive = SimSwerveDrive.create(world, config, startPose);
  }
}
```

## 2. Every simulation period

The motor controllers own the voltages. Feed them encoder values from the sim, take their applied
voltages back, step the world once.

```java
@Override
public void simulationPeriodic() {
  for (int m = 0; m < 4; m++) {
    drive.setModuleVoltages(m, driveMotorVolts(m), steerMotorVolts(m)); // from your motor sim states
  }
  world.step(0.020);                                                    // 5 physics substeps

  RoboRioSim.setVInVoltage(drive.getBatteryVoltage());                  // battery sag for the whole robot
  // ...feed encoders (below), gyro, and telemetry
}
```

### CTRE Phoenix 6 (TalonFX + CANcoder + Pigeon 2)
```java
SwerveRobot robot = drive.getRobot();
TalonFXSimState driveSim = driveMotor.getSimState();
driveSim.setSupplyVoltage(drive.getBatteryVoltage());
double driveVolts = driveSim.getMotorVoltage();                // input to frcsim
// after world.step():
driveSim.setRawRotorPosition(Units.radiansToRotations(robot.driveRotorPosition(m)));
driveSim.setRotorVelocity(Units.radiansToRotations(robot.driveRotorVelocity(m)));
steerSim.setRawRotorPosition(Units.radiansToRotations(robot.steerRotorPosition(m)));
cancoder.getSimState().setRawPosition(Units.radiansToRotations(robot.steerAngle(m)));
pigeon.getSimState().setRawYaw(Math.toDegrees(drive.getGyroYawRadians()));
```
Invert signs to match your motor inversion settings: frcsim's positive drive voltage rolls the wheel
forward, and positive steer voltage turns the module counterclockwise.

### REV (SparkMax/SparkFlex)
```java
SparkSim driveSim = new SparkSim(driveSpark, DCMotor.getNeoVortex(1));
double driveVolts = driveSim.getAppliedOutput() * drive.getBatteryVoltage();
// after world.step(): SparkSim.iterate expects mechanism velocity in RPM (after conversion factors)
double wheelRpm = Units.radiansPerSecondToRotationsPerMinute(robot.driveRotorVelocity(m) / driveGearRatio);
driveSim.iterate(wheelRpm * velocityConversionFactor, drive.getBatteryVoltage(), 0.020);
```

### Plain WPILib (no vendor sim)
Run your own PID on the sim encoders and call `setModuleVoltages` directly. For stiff steering, step the
world in smaller increments (e.g. `world.step(0.004, 1)` five times per period) so your loop runs at 250 Hz
like a motor controller's onboard loop.

## 3. Odometry vs ground truth

- `drive.getModulePositions()` and `drive.getGyroYaw()` are **sensor values**: wheel slip shows up as drift,
  so feed them to your `SwerveDrivePoseEstimator` just like on the real robot.
- `drive.getPose()` / `getPose3d()` is **ground truth**: log it next to your estimate in AdvantageScope to see
  estimator error.
