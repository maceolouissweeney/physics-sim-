# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow semantic versioning.

## [Unreleased]

### Added
- Project plan (`CLAUDE.md`), implementation plan, building guide, ADR-0001.
- Native CMake project with presets for Windows (MSVC), Linux, and macOS; pinned Jolt Physics v5.6.0
  and GoogleTest v1.18.0 with SHA-256 verification.
- Core `World`: Jolt runtime lifecycle, collision layers, single-threaded/thread-pool job systems,
  fixed-substep stepping.
- C ABI v1 (`frcsim_c.h`): version, status/error reporting, world create/step/destroy.
- JNI bindings and Java `frcsim-core` module: `FrcSim`, `SimWorld`, `WorldConfig`, native loader with
  ABI check.
- `frcsim-native` Gradle module: packages native libraries as WPILib JNI zips (release + debug
  classifiers), publishes to a local Maven repo, generates the `frcsim.json` vendordep (ADR-0002).
- `examples/smoke-robot`: stock WPILib 2026 project that verifies the vendordep end to end.
- GitHub Actions CI workflow (native matrix incl. macOS universal + Linux arm64, ASan/UBSan, Java,
  smoke robot). Not yet run.
- **Phase 1 (world + game pieces):**
  - Materials with per-pair friction/restitution overrides, applied per contact; standard FRC
    materials (`carpet`, `polycarbonate`, `aluminum`, `bumper`, `foam`; estimates).
  - Static field geometry (ground, box, Z-axis cylinder, convex hull) and field bounds.
  - Field JSON loader (`frcsim.field/1`, `docs/reference/field-json.md`) and `fields/test-flat`.
  - Game piece types (sphere, cylinder, box) and a structure-of-arrays piece pool with body recycling
    and a state machine (on field, airborne, in robot, scored, out of bounds).
  - Kinematic bodies and force-limited driven bodies (traction-limited robot stand-ins).
  - Dedicated broadphase layer for pieces: 504 sleeping pieces cost 0.025 ms per period (was 0.5 ms).
  - C ABI: shared-memory world stats, materials, field, pieces (zero-copy position/state buffers),
    kinematic bodies.
  - Java: `WorldStats`, `Materials`, `Field`, `GamePieces`, `GamePieceTypeSpec`, `PieceState`,
    `KinematicBodies`, `CapacityExceededException`.
  - `frcsim_bench` scenario runner and a zero-allocation stepping test.
  - `frcsim-jmh` module: JMH benchmarks of JNI crossing, stats reads, and piece position reads.
  - Performance report `docs/perf/phase1.md`.

### Changed
- Java `WorldConfig` is now built with `WorldConfig.builder()` (was a record with `with*` methods).
- Java `SimWorld.timeSeconds()` reads shared memory instead of calling into native code.
- Default physics worker threads changed from 0 (single-threaded) to 2: dense piles run about 2× faster (D24).
