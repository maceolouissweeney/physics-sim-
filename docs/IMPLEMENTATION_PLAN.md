# Implementation Plan

This is the living, task-level plan. `CLAUDE.md` holds the architecture and research; this file
tracks **what gets built, in what order, and how we know each piece is done.** Check off tasks as they
land. Record deviations in the Decision Log (§2).

Legend: `[ ]` todo · `[~]` in progress · `[x]` done · `(!)` blocked or needs a decision.

**Definition of done (every task):** code + tests + docs updated. Hot-path changes also include
`frcsim_bench` numbers. Every task also adds a `CHANGELOG.md` entry under *Unreleased*.

---

## 1. Pinned versions

| Component | Version | Pin |
|---|---|---|
| Jolt Physics | v5.6.0 | commit `e77f175595e64cb44218cc9d9d56fc365ad0e36a` + SHA-256 in `native/cmake/FrcsimDependencies.cmake` |
| GoogleTest | v1.18.0 | tarball SHA-256 |
| Google Benchmark | v1.9.5 | tarball SHA-256 |
| nlohmann/json | v3.12.0 | release tarball SHA-256 |
| CMake | ≥ 3.25 | |
| MSVC | VS 2022 (17.x) | Windows toolchain |
| JDK | 17 (WPILib 2026 bundled Temurin) | `--release 17` |
| Gradle | 8.11 (matches GradleRIO 2026.2.1) | wrapper |
| WPILib | 2026.2.1 (primary), 2027 alphas (CI later) | |

---

## 2. Decision log

| ID | Decision | Why |
|---|---|---|
| D1 | Java package and Maven group are the placeholder **`org.frcsim`** | Needs a name to start; rename before first publish (tracked in §10). |
| D2 | **One shared library `frcsim`** exports both the C ABI and the JNI entry points | One file to package and load. The JNI glue only calls the C ABI, so a separate C++/FFM consumer can still use the same library. |
| D3 | **Static MSVC runtime (`/MT`)** for all native code | Jolt defaults to it; a self-contained DLL avoids the "incorrect MSVC runtime" JVM load failures WPILib warns about. |
| D4 | **Z-up world**, gravity along −Z (9.80665 m/s²), SI units | Matches WPILib field coordinates; Jolt defaults to Y-up, so set gravity explicitly. |
| D5 | Native install tree mirrors WPILib zips: **`<os>/<arch>/shared/<lib>`** | Same layout GradleRIO extracts from `wpiutil-cpp-*-windowsx86-64.zip`. |
| D6 | Field JSON is parsed **natively** (nlohmann/json) | One parser shared by the Java API, benchmarks, and C++ tests. |
| D7 | Game-piece bodies set Jolt `mLinearDamping`/`mAngularDamping` to **0** | Jolt's default 0.05 damping is fake air drag; our aero model owns air resistance and rolling resistance. |
| D8 | Jolt build: profiler and debug renderer **off**, **SSE4.2 baseline** (no AVX/AVX2/LZCNT/F16C) | Performance and compatibility with low-end school laptops (see CLAUDE.md §3). |
| D9 | `World` worker threads: `0` = single-threaded job system (default), `N` = thread pool | Deterministic and fastest at small scale; revisit with Phase 1 benchmarks. |
| D10 | Two gtest executables: `frcsim_core_tests` (static core, internals) and `frcsim_capi_tests` (links only the shared library) | Avoids two copies of Jolt globals in one process; the C ABI test exercises exactly what Java sees. |
| D11 | Local Windows builds use the **Visual Studio 17 2022 generator** (no Developer Prompt needed); CI/Linux/macOS use Ninja | Lets tooling invoke CMake from a plain shell. |
| D12 | macOS universal binary is produced by building `arm64` and `x86_64` separately and running `lipo` | Jolt's per-arch SIMD flags make a single-pass universal build fragile. |
| D13 | google-java-format pinned to **1.28.0** | 1.29.0+ requires JDK 21 to run; Gradle runs on the WPILib 2026 JDK 17. |
| D14 | Piece bodies set `mMaxAngularVelocity` per piece type (default **500 rad/s**) | Jolt's default of 0.25π·60 ≈ 47 rad/s would clamp shooter backspin (~130 rad/s for a hooded fuel shot). |
| D15 | `PhysicsSettings::mMinVelocityForRestitution` lowered from Jolt's 1.0 m/s to **0.2 m/s** (configurable) | Jolt zeroes restitution for impacts slower than this; fuel visibly bounces at lower speeds. 0.2 m/s matches maple-sim's tuned bounce threshold. |
| D16 | Materials: an 8-bit material id in each body's user data + a dense 64×64 pair table, applied in `ContactListener::OnContactAdded/Persisted` | O(1) per contact, supports pair overrides (foam-on-carpet ≠ foam-on-polycarbonate), no per-subshape materials needed. |
| D17 | Piece bodies are created on first spawn and **recycled per type**: despawned/intaken pieces are removed from the broadphase, never destroyed | Zero allocations in steady state; re-adding a body is cheap. |
| D18 | The frcsim API uses **Z-axis cylinders**; Jolt cylinders (Y-axis) are wrapped in a `RotatedTranslatedShape` | Z-up world (D4); one conversion point in `world/shapes.*`. |
| D19 | Field statics use a **5 mm convex radius** (pieces use Jolt's default where it fits) | Jolt's 5 cm default rounds wall and rim edges enough to change bounces. |
| D20 | Google Benchmark is not fetched until micro-benchmarks exist; the Phase 1 scenario runner uses `std::chrono` | Avoid unused dependencies. |
| D21 | `FRCSIM_ABI_VERSION` stays **1** until the first published release | Nothing external consumes the ABI yet; Java and native always come from the same commit. |
| D22 | Zero-allocation rule applies to **frcsim code**; Jolt's broadphase rebuild may make ≤ 2 transient allocations per substep | Measured: `QuadTree::UpdatePrepare` does `new NodeID[numBodies]` each update (Jolt v5.6.0). Bounded, freed immediately, and no GC is involved. Test enforces the budget. A pooled Jolt allocator can remove it later if profiling shows cost. |
| D23 | `WorldConfig` (Java) is an immutable class with a **builder** instead of a record | 13 settings and growing; a builder keeps call sites readable and allows adding fields without breaking callers. |
| D24 | Default **`workerThreads = 2`** (supersedes D9's single-threaded default) | Measured on i7-10610U: dense 360-piece pile 2.77 ms single-threaded vs 1.45 ms with 2 workers; 3–4 workers are slower. Jolt's deterministic mode gives the same results for any thread count. Idle cost rises from 0.03 to 0.10 ms, well within budget. |
| D25 | Solver iterations stay at Jolt defaults (10 velocity / 2 position) | Sweeping 4–10 / 1–2 changed dense-pile timing by < 5%: collision detection dominates, so fewer iterations buy nothing. |

---

## 3. Phase 0 — Prove the pipeline ✅

Goal: a WPILib 2026 robot project installs the vendordep, and `FrcSim.version()` runs in desktop sim
and unit tests, loading a Jolt-linked native library.

| ID | Task | Key files | Verification |
|---|---|---|---|
| [x] P0.1 | Repo scaffolding: `.gitignore`, `.gitattributes`, `.editorconfig`, `.clang-format`, `README.md`, `CHANGELOG.md`, docs index | root, `docs/README.md` | files exist |
| [x] P0.2 | CMake project: presets, platform detection, compiler options, pinned dependencies (Jolt + GoogleTest) with SHA-256 | `native/CMakeLists.txt`, `native/CMakePresets.json`, `native/cmake/*.cmake` | configures + builds with MSVC 19.44, no warnings |
| [x] P0.3 | Core `World`: Jolt runtime refcount init, layers, temp allocator, job system, fixed-substep `step()` | `native/src/world/*` | 8 `frcsim_core_tests` pass |
| [x] P0.4 | C ABI v1: version, ABI version, status strings, thread-local last error, world create/destroy/step/time | `native/include/frcsim/frcsim_c.h`, `native/src/capi/frcsim_c.cpp` | 10 `frcsim_capi_tests` pass |
| [x] P0.5 | JNI glue (calls C ABI only), status → Java exception mapping | `native/src/jni/frcsim_jni.cpp` | Java tests (P0.6) |
| [x] P0.6 | Java Gradle build (wrapper 8.11), `frcsim-core`: `NativeLoader` (ABI check), `FrcSim`, `SimWorld` (AutoCloseable + Cleaner), `WorldConfig` record | `java/**` | 9 JUnit tests pass (6 via JNI); Spotless + Javadoc clean |
| [x] P0.7 | Native packaging: zip per platform classifier (+ `debug` twin) from install tree; `maven-publish` to local file repo | `java/frcsim-native/build.gradle` | zip layout `windows/x86-64/shared/frcsim.dll` |
| [x] P0.8 | Vendordep JSON generated from a template with version substitution | `vendordep/frcsim.json.in`, `:frcsim-native:generateVendordep` | consumed by WPILib 2026 project (P0.9) |
| [x] P0.9 | End-to-end smoke: stock WPILib 2026 Java template project using the local Maven repo + vendordep | `examples/smoke-robot/` | `gradlew test` 2/2 pass; `gradlew build` passes; `roborioRelease` resolves without frcsim, `nativeRelease` includes it |
| [~] P0.10 | CI workflow: native matrix (Windows, Linux x64/arm64, macOS universal), ASan/UBSan, Java on 3 OSes, smoke robot | `.github/workflows/ci.yml` | actionlint 1.7.12 clean; (!) not run until a GitHub remote exists |
| [x] P0.11 | Docs: building guide, ADR-0001 (C++ core + Jolt), ADR-0002 (native packaging) | `docs/building.md`, `docs/adr/*` | written |

**Gate: met on Windows (2026-09-12).** Linux/macOS are verified when CI first runs (P0.10).
Not verified: `simulateJava` with the GUI (unit tests exercise the same GradleRIO JNI extraction).

---

## 4. Phase 1 — World and game pieces at scale ✅

Goal: 504 fuel-sized spheres on a walled flat field, with a kinematic "plow" pushing through them, within the §7
budgets in CLAUDE.md.

| ID | Task | Key files | Verification |
|---|---|---|---|
| [x] P1.1 | `PhysicsSettings` tuning surface (solver iterations, sleep thresholds, restitution threshold) exposed via config | `world/world.h` (`WorldConfig`) | config validation tests |
| [x] P1.2 | Body user-data encoding: `kind:4 \| material:8 \| reserved:20 \| index:32` | `world/body_tag.h` | round-trip unit tests |
| [x] P1.3 | Materials: per-material friction/restitution + **pair override table** applied in `ContactListener::OnContactAdded/Persisted` | `world/materials.*`, `world/contact_listener.h` | bounce apex ≈ e²·h test; pair override removes bounce |
| [x] P1.4 | Static field geometry (ground, box, Z-cylinder, convex hull), bounds; JSON field loader; `fields/test-flat/field.json` | `field/*`, `world/shapes.*`, `fields/` | wall stops ball; cylinder axis; JSON load + error location tests |
| [x] P1.5 | `PieceTypeRegistry` (sphere/cylinder/box, mass, material, max angular velocity) | `pieces/piece_type.*` | validation tests (aero params deferred to Phase 4) |
| [x] P1.6 | `PiecePool`: bodies created on first spawn and **recycled per type** (D17), SoA arrays, state machine, batch add via `AddBodiesPrepare/Finalize`, out-of-bounds detection | `pieces/piece_pool.*` | lifecycle/capacity/recycling tests; zero-allocation stepping test (D22) |
| [x] P1.7 | Thread-safe fixed-capacity event buffer (contact callbacks run on job threads) | `util/event_buffer.h` | concurrent push test (consumers arrive with sensors, P3.2) |
| [x] P1.8 | Shared buffers: world stats block (layout is ABI, `static_assert`ed natively, verified from Java) + piece position/state buffers wrapped once with `NewDirectByteBuffer` | `capi/frcsim_c.cpp`, `WorldStats.java`, `GamePieces.java` | Java reads settled positions; layout test |
| [x] P1.9 | Kinematic body API (scripted obstacles) | `world/kinematic_bodies.*` | reaches target; pushes piece |
| [x] P1.9b | **Driven bodies**: force-limited dynamic robot stand-ins via `PhysicsStepListener` (kinematic plows crushed piles and ejected pieces) | `world/driven_bodies.*` | force-limited acceleration; stalls at wall; pushes pile without launching pieces. Native-only for now |
| [x] P1.10 | `frcsim_bench` scenario runner: `empty_field`, `sleeping_504`, `awake_360_plow`, `full_504_plow`; thread/solver flags; p50/p95/p99; JSON output; out-of-bounds diagnostics | `native/bench/*` | prints table |
| [x] P1.11 | Performance tuning pass + write-up: piece broadphase layer (20× idle speedup), driven bodies, thread and solver sweeps (D24, D25) | `docs/perf/phase1.md` | budgets met (see gate) |
| [x] P1.12 | Java API: `Field.loadJson`, `Materials`, `GamePieces` (zero-alloc accessors), `KinematicBodies`, `WorldStats`; JMH module | `java/frcsim-core/**`, `java/frcsim-jmh/**` | 17 JUnit tests; JMH: JNI round trip 28 ns, 504-position copy 216 ns, empty-world step 39 µs (upper bound on overhead) |
| [x] P1.13 | Docs: world & pieces architecture, field JSON schema v1 | `docs/architecture/world-and-pieces.md`, `docs/reference/field-json.md` | written |

**Gate: met on Windows (2026-09-12)** with the default 2 worker threads on the i7-10610U laptop:
`sleeping_504` p50 0.10 ms (≤ 0.5), `awake_360_plow` p50 1.45 ms (≤ 2.0), JNI overhead far below 50 µs.
Known gaps (tracked in `docs/perf/phase1.md`): single-threaded dense piles 2.77 ms; `awake_360_plow` p95 2.8 ms.
Driven bodies are native-only (Java API arrives with robots in Phase 2).

---

## 5. Phase 2 — Swerve drivetrain

**Spike findings already confirmed from Jolt v5.6.0 headers:** `VehicleController` is subclassable
(`ConstructWheel`, `PreCollide`, `PostCollide`, `SolveLongitudinalAndLateralConstraints`, save/restore
state). `Wheel` exposes `SetSteerAngle`, `SetAngularVelocity`, contact point velocity, normal,
longitudinal and lateral directions, suspension lambda (→ normal force), and
`SolveLongitudinalConstraintPart` / `SolveLateralConstraintPart(min, max impulse)`. So per-wheel steer
plus friction-limited impulses fit the design.

| ID | Task | Key files | Verification |
|---|---|---|---|
| [ ] P2.1 | Spike: read `Wheel.cpp` / `WheeledVehicleController.cpp` for the longitudinal velocity target and wheel spin integration; prototype a 4-wheel swerve on flat ground, Z-up (`VehicleConstraintSettings::mUp/mForward`), cylinder-cast collision tester | `drive/swerve_vehicle_controller.*` | ADR-0003 written with findings |
| [ ] P2.2 | `DcMotor`: derive R, Kv, Kt from (nominal V, stall torque, stall current, free current, free speed); stator/supply current limits; brake/coast | `drive/dc_motor.*` | stall current, free speed, limit tests |
| [ ] P2.3 | `Battery` / electrical bus per robot (sag, brownout flag) | `drive/battery.*` | sag formula test |
| [ ] P2.4 | `SwerveModule`: drive/steer gearing, J_eff with reflected rotor inertia, efficiency, float64 encoders | `drive/swerve_module.*` | encoder integration test |
| [ ] P2.5 | Tire model: `μ(s) = μ_peak·tanh(|s|/s₀)` with combined friction circle → impulse bounds per substep | `drive/tire_model.*` | traction-limited acceleration ≈ μg |
| [ ] P2.6 | Steer dynamics + scrub torque | `drive/swerve_module.*` | steer step response tests |
| [ ] P2.7 | Chassis body: compound shape (bumpers + frame), mass/CoG/inertia overrides, bumper material | `drive/chassis.*` | mass properties test |
| [ ] P2.8 | Robot C ABI + buffers (inputs: voltages; outputs: rotor pos/vel, currents, module angles, pose, twist, gyro) | `capi/robot.*` | C ABI tests |
| [ ] P2.9 | Sensors: gyro (yaw/pitch/roll + rates), optional seeded noise and drift | `sensors/*` | noise-off exactness test |
| [ ] P2.10 | Validation suite (free speed, μg accel limit, pushing match, stall current, rotate in place) + 10k-step NaN fuzz, 1–10 substeps | `tests/drive/*` | all pass |
| [ ] P2.11 | Java: `SwerveConfig`/`ModuleConfig` records, `SwerveRobot`; new `frcsim-wpilib` module (DCMotor, Pose2d/3d, SwerveModuleState adapters; `RobotBase.isReal()` guard) | `java/frcsim-wpilib/**` | JUnit |
| [ ] P2.12 | Docs: swerve model (equations + parameter guide), quickstart, CTRE/REV integration recipes | `docs/models/swerve.md`, `docs/guides/*` | reviewed |

**Gate:** validation suite green; bench scenario with 2 robots + 360 awake pieces within budget.

---

## 6. Phase 3 — Field elements and the REBUILT arena

| ID | Task | Verification |
|---|---|---|
| [ ] P3.1 | Field JSON schema v2: heightfield/mesh statics (bumps/ramps), sensor volumes, piece spawn lists, materials | schema doc + loader tests |
| [ ] P3.2 | Sensor volumes → events (enter/exit per piece/robot) | tests |
| [ ] P3.3 | Author `fields/2026-rebuilt/field.json` from official field drawings (hubs, bumps, trenches, tower, depot/outpost) | overlay check against AdvantageScope field model |
| [ ] P3.4 | `frcsim-games` module: `Arena` abstraction, `Rebuilt2026` (504 placements, scoring, hub state, human-player re-entry) | JUnit rule tests |
| [ ] P3.5 | Telemetry: struct-array publishers (`Translation3d[]` pieces, `Pose3d[]` robots), AdvantageKit helpers | AdvantageScope visual check |
| [ ] P3.6 | Example AdvantageKit swerve robot project | runs in sim |
| [ ] P3.7 | Docs: authoring a field, season arena guide | reviewed |

**Gate:** robot drives over bumps, fuel scatters, budgets hold with the full REBUILT arena.

---

## 7. Phase 4 — Shooting and mechanisms

| ID | Task | Verification |
|---|---|---|
| [ ] P4.1 | Aero pass over the airborne index list: drag, Magnus `C_L(S)` table, spin decay | unit tests on force values |
| [ ] P4.2 | CCD on/off switching by piece state (`LinearCast` when airborne) | fast ball vs thin rim no tunneling |
| [ ] P4.3 | Flywheel shooter: motor + inertia, energy extraction per shot, grip efficiency, spin from surface-speed differential, seeded dispersion | RPM dip & energy conservation tests |
| [ ] P4.4 | Shot queue + exit pose (robot ⊕ turret ⊕ hood) + chassis velocity inheritance | shoot-on-the-move test |
| [ ] P4.5 | Intake zones (sensor-based acceptance rules) + optional surface-velocity rollers | intake rules tests |
| [ ] P4.6 | Indexer/hopper queue with transit time and feed-rate limit | tests |
| [ ] P4.7 | Java `ShotCalculator` sharing the aero equations (RK4) | matches native trajectories within 1 cm |
| [ ] P4.8 | Docs: projectile & mechanism models | reviewed |

**Gate:** drag-free trajectory matches analytic exactly; drag trajectory within 1 cm of RK4 reference;
60 airborne + 504 pieces + 6 robots ≤ 4 ms/period.

---

## 8. Phase 5 — API hardening and ecosystem

- [ ] P5.1 API review pass; Javadoc on all public types; `@PeriodicSafe` annotation for zero-alloc methods
- [ ] P5.2 Onboard motor-controller closed-loop models (velocity/position PID + FF at substep rate)
- [ ] P5.3 WPILib 2027 (Java 25) CI job + adapter module if package renames require it
- [ ] P5.4 Evaluate Java FFM binding on the same C ABI (benchmark vs JNI)
- [ ] P5.5 Publishing pipeline: static Maven repo (GitHub Pages / R2), tagged releases, semver policy
- [ ] P5.6 Examples: basic swerve, AKit + Phoenix 6, shooter, multi-robot; docs site

## 9. Phase 6 — Calibration and validation against reality

- [ ] P6.1 `tools/fit_drivetrain.py`: fit μ, s₀, efficiency from AdvantageKit logs
- [ ] P6.2 Fuel drop/roll/incline test protocol + fitting script (restitution, friction, rolling resistance)
- [ ] P6.3 High-speed video trajectory fitting for C_d and C_L(S)
- [ ] P6.4 "Sim vs real" report; replace all `CALIBRATE:` constants

---

## 10. Open items needing decisions

- (!) Final project/package name (replaces `org.frcsim`, D1)
- (!) License (MIT suggested, compatible with Jolt MIT, GoogleTest BSD-3, nlohmann MIT)
- (!) GitHub remote & Maven hosting location (unblocks P0.10, P5.5)
