# frcsim

High-performance physics simulation for FRC robots: realistic swerve drivetrains, collisions with field
elements, hundreds of game pieces, and projectile flight with drag and spin. A C++ core
([Jolt Physics](https://github.com/jrouwe/JoltPhysics)) exposed to Java robot projects as a WPILib vendordep.

> **Status: early development.** Phases 0–1 are done: packaging, world, field, and game pieces at scale.
> Swerve drive (Phase 2) is next. Not usable for robot code yet. "frcsim" and `org.frcsim` are placeholder
> names.

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

  float[] positions = new float[3 * world.pieces().capacity()];
  for (int i = 0; i < 50; i++) {
    world.step(0.020);                                   // one robot period, 5 physics sub-steps
    int count = world.pieces().copyPositions(positions);  // zero-copy source, no allocation
  }
}
```

Performance on a 2020 15 W laptop (i7-10610U): 504 resting pieces cost 0.1 ms per robot period.
360 pieces being plowed by two robots cost 1.45 ms (p50).
