# frcsim documentation

| Document | Purpose |
|---|---|
| [../CLAUDE.md](../CLAUDE.md) | Architecture, research on what to model, performance rules, conventions |
| [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) | Task-level plan, progress, pinned versions, decision log |
| [building.md](building.md) | Toolchain setup, native + Java builds, troubleshooting |
| [architecture/world-and-pieces.md](architecture/world-and-pieces.md) | World ownership, collision layers, materials, field, piece pool, kinematic and driven bodies |
| [reference/field-json.md](reference/field-json.md) | `frcsim.field/1` field definition schema |
| [perf/phase1.md](perf/phase1.md) | Phase 1 benchmark results, tuning findings, next optimization candidates |
| [adr/](adr/) | Architecture Decision Records |

## Architecture Decision Records

| ADR | Title |
|---|---|
| [0001](adr/0001-cpp-core-jolt-jni.md) | C++ core on Jolt Physics, exposed via C ABI + JNI |
| [0002](adr/0002-native-packaging.md) | Native library packaging as WPILib JNI zips + vendordep |
| [0003](adr/0003-swerve-tire-solver.md) | Swerve drivetrain as a custom Jolt `VehicleController` |

Smaller decisions are recorded in the decision log in [IMPLEMENTATION_PLAN.md §2](IMPLEMENTATION_PLAN.md).

## Planned documents
- `models/`: projectiles and mechanisms (Phase 4). The swerve model is done:
  [models/swerve.md](models/swerve.md)
- `guides/`: quickstart, CTRE Phoenix 6 integration, authoring a field
