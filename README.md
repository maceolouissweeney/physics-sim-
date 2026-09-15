# frcsim

High-performance physics simulation for FRC robots: realistic swerve drivetrains, collisions with field
elements, hundreds of game pieces, and projectile flight with drag and spin. A C++ core
([Jolt Physics](https://github.com/jrouwe/JoltPhysics)) exposed to Java robot projects as a WPILib vendordep.

> **Status: early development.** Done: packaging (vendordep), world, field, game pieces at scale, and swerve
> drivetrains (motors, battery, tire slip, load transfer, pushing, sensors) with WPILib adapters. Targets CTRE
> Phoenix 6 hardware (REV is not supported). Next: REBUILT
> field elements (Phase 3), then shooting and intakes (Phase 4). Not published yet; "frcsim" and
> `org.frcsim` are placeholder names.

## Documentation

- [Architecture, research & conventions](CLAUDE.md)
- [Implementation plan & progress](docs/IMPLEMENTATION_PLAN.md)
- [Building from source](docs/building.md)
- [Performance results](docs/perf/phase1.md)
- [All docs](docs/README.md)

## Quick look (current API)

```java
try (SimWorld world = SimWorld.create()) {
  world.field().loadJson(Path.of("fields/test-flat/field.json"));
  GamePieceType fuel = world.pieces().findType("fuel").orElseThrow();
  world.pieces().spawn(fuel, 8.0, 4.0, 1.0);

  float[] positionsXyzMeters = new float[3 * world.pieces().capacity()];
  for (int i = 0; i < 50; i++) {
    world.step(0.020);                                                   // one robot period, 5 sub-steps
    int count = world.pieces().copyPositionsMeters(positionsXyzMeters);  // no allocation
  }
}
```

### Swerve robot (WPILib types)

```java
SwerveModuleConfig module = new SwerveModuleConfig();        // defaults: SDS MK4i L2, Kraken X60
module.driveMotor = WpilibMotors.fromDCMotor(DCMotor.getKrakenX60Foc(1), 6.0e-5);
SimSwerveDrive drive =
    SimSwerveDrive.create(world, SwerveDriveConfig.rectangular(0.55, 0.55, module), new Pose2d(2, 4, Rotation2d.kZero));

drive.setModuleCommandVolts(0, 6.0, 0.0);                      // feed from your TalonFX sim states
world.step(0.020);
SwerveModulePosition[] odometry = drive.getModulePositions();  // from simulated encoders: slip causes drift
Pose2d truth = drive.getPose();                                // ground truth for comparison
```
Guide: [docs/guides/swerve-quickstart.md](docs/guides/swerve-quickstart.md). Model:
[docs/models/swerve.md](docs/models/swerve.md).

Performance on a 2020 15 W laptop (i7-10610U): 504 resting pieces cost 0.1 ms per robot period.
360 pieces being plowed by two robots cost 1.45 ms (p50).
