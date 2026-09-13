# frcsim — High-Performance FRC Physics Simulation

> Working name `frcsim`. Status: **Phases 0–1 complete (verified on Windows); Phase 2 (swerve) next.**
> This file is the source of truth for architecture, research, and conventions. Update it when a
> decision changes. Task-level progress, pinned versions, and the decision log live in
> [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md); build setup lives in
> [`docs/building.md`](docs/building.md).

**Working rules for this repo**
- Keep docs current as you go: tick tasks in the implementation plan, add a `CHANGELOG.md` entry,
  record new decisions in the decision log or an ADR (`docs/adr/`).
- A task isn't done until native tests (`ctest`), Java tests (`./gradlew build`), and, when packaging
  is touched, the smoke robot (§10) all pass.
- The C ABI (`native/include/frcsim/frcsim_c.h`) is the only native surface bindings may use. Bump
  `FRCSIM_ABI_VERSION` and `NativeLoader.EXPECTED_ABI_VERSION` together on breaking changes.

## 1. Goals

A physics simulation library for FRC robot code (desktop simulation) that models:

1. **Swerve drivetrains realistically**: motor electrical model, battery sag, tire slip, traction
   limits, load transfer, robot-robot pushing, and odometry drift caused by slip.
2. **Collisions** with field borders, field elements (including 3D features like bumps and ramps), and
   other robots.
3. **Large numbers of game pieces**: target is **504 REBUILT fuel balls on the field, with 360+ moving at
   once**, at well above real time.
4. **Projectiles**: many pieces in flight at once, with gravity, drag, Magnus lift, and spin, landing
   back into the physics world. Rapid-fire shooting must show flywheel speed dropping between shots.
5. **Use from a Java robot project** as a WPILib vendordep.

**Priorities, in order:** (1) performance, (2) realistic swerve and game-piece physics,
(3) maintainability and extensibility, since the game changes every year.

**Non-goals:** running on the robot controller (roboRIO/Systemcore); rendering (AdvantageScope does
that); soft-body or deformable simulation beyond simple contact compliance; electrical simulation
beyond the battery and motors.

### Why not just use maple-sim?
maple-sim (the current community standard) uses **dyn4j**, a 2D engine, running at 250 Hz. Its
projectiles are **kinematic with no air drag**, using gravity fudged to 11 m/s² to compensate. Users
report roughly **7–10 fps in REBUILT with fuel mostly idle and 1–2 fps when fuel is disturbed**. The
2D engine can't handle ramps, bumps, stacking, or pieces bouncing off a hub, and its object-heavy Java
design is a poor fit for hundreds of bodies. We keep its good ideas (arena abstraction, intake
concepts, field mirroring) and fix these limits.

---

## 2. Architecture Decision: C++ core + thin Java binding

| Option | Verdict | Reason |
|---|---|---|
| **C++20 core + Jolt Physics + batched JNI** | ✅ **Chosen** | SIMD, no GC or JIT warm-up, full control of memory layout. Jolt is a fast, MIT-licensed 3D engine with sleeping, CCD, sensors, surface velocities, and an extensible vehicle system. The core is independent of WPILib, so it survives WPILib's yearly API breaks and could later serve C++ and Python teams. |
| Pure Java, custom solver (SoA `double[]`) | Fallback | Viable for about 500 spheres, but no dependable SIMD (the Vector API is still incubating), GC and JIT variance, and we would have to write and debug a full 3D contact solver ourselves. |
| jolt-jni (existing Java bindings) | ❌ | Fine-grained 1:1 API means per-body JNI calls every tick, and JNI chatter would dominate the frame. |
| dyn4j / JBox2D / Box2D v3 | ❌ | 2D only. |
| PhysX | ❌ | Heavy build and dependency, no advantage at this scale. |
| MuJoCo | ❌ | Excellent contact modeling for articulated robots, but not built for hundreds of free bodies. |
| Bullet | ❌ | Slower and less actively developed than Jolt. |

**Key rule: the Java↔native boundary is crossed a constant number of times per robot period
(target: 1 `step()` call), never once per body.** Bulk state is exchanged through shared direct
`ByteBuffer`s.

**Escape hatch:** game pieces go through an internal `PieceSolver` seam. If profiling shows Jolt's
general pipeline is the bottleneck for sphere-only pieces, a specialized sphere solver (spatial hash
plus soft-step or XPBD) can replace it without touching the drivetrain or the Java API. Don't build
this unless benchmarks demand it.

---

## 3. Tech Stack

### Native core (`native/`)
| Concern | Choice |
|---|---|
| Language | **C++20** (MSVC 2022, Clang 16+, GCC 11+). Move to C++23 when all three CI toolchains support it. WPILib 2027 uses C++23. |
| Physics engine | **Jolt Physics** (MIT), pinned to a release tag, statically linked. Single precision (the field is ~16.5 m, so double precision isn't needed). Optional `JPH_CROSS_PLATFORM_DETERMINISTIC` build for regression tests. |
| Build | **CMake 3.25+** with presets (`CMakePresets.json`), Ninja; dependencies via `FetchContent` pinned by commit hash |
| Math | Jolt's math types (`Vec3`, `Quat`, `Mat44`) inside the core. No Eigen or GLM. |
| Unit tests | **GoogleTest** |
| Benchmarks | **Google Benchmark** plus a standalone scenario runner (`frcsim_bench`) that prints ms/tick percentiles |
| Profiling | **Tracy** (compile-time opt-in, `FRCSIM_TRACY=ON`); Superluminal/VTune/Instruments ad hoc |
| Sanitizers | ASan + UBSan in the Linux CI debug job |
| Formatting / lint | clang-format (checked in `.clang-format`), clang-tidy (`.clang-tidy`) |
| SIMD baseline | x86-64: **SSE4.2** baseline, because school laptops (Celeron/Pentium Silver) often lack AVX2. Add an AVX2 variant, chosen at load time via CPUID, **only if benchmarks show >15% gain.** Apple Silicon: NEON. |
| JNI | Hand-written thin JNI layer over the internal C ABI (`frcsim_c.h`). No SWIG or JavaCPP. |

### Java binding (`java/`)
| Concern | Choice |
|---|---|
| Language level | `--release 17` (WPILib 2026 = Java 17). WPILib 2027 (Systemcore) uses **Java 25**: add a CI job against 2027 betas once they stabilize. |
| Build | **Gradle** (wrapper version matches GradleRIO's), `java-library` + `maven-publish` |
| Modules | `frcsim-core` (**no WPILib dependency**: primitive and own types, native loading, buffers) · `frcsim-wpilib` (adapters to `Pose2d/3d`, `DCMotor`, `SwerveModuleState`, struct publishers). Per-season adapter module only if WPILib APIs diverge (e.g., 2027 package renames). |
| Game/season layer | `frcsim-games` (Java): arena definitions, scoring rules, and field presets per season, kept in Java so teams can read and extend them |
| Native loading | WPILib `RuntimeLoader`/`CombinedRuntimeLoader` pattern (natives extracted by GradleRIO `jniDependencies`) |
| Tests | JUnit 5 |
| Binding benchmarks | JMH (JNI overhead per `step()`, buffer read cost) |
| Formatting | Spotless + google-java-format **1.28.0** (newest that runs on JDK 17; 1.29+ needs JDK 21) |
| Telemetry | WPILib NT4 **struct arrays** (`Pose3d[]`, `Translation3d[]`) for AdvantageScope; AdvantageKit-friendly (`LoggableInputs` helpers) |
| CTRE/REV integration | Examples feeding `TalonFXSimState` / `SparkSim` from our rotor outputs |

### Distribution and CI
| Concern | Choice |
|---|---|
| Package | WPILib **vendordep JSON**: `javaDependencies` + `jniDependencies` with `skipInvalidPlatforms: true` |
| Native platforms | `windowsx86-64`, `linuxx86-64`, `osxuniversal` (arm64 + x86_64), `linuxarm64` optional. **No `linuxathena`/Systemcore**, since the sim never runs on the robot. |
| Maven hosting | Static Maven repo on GitHub Pages or R2. Not GitHub Packages, which requires auth to read. |
| CI | GitHub Actions matrix (windows-2022, ubuntu-22.04, macos-14) → build and test natives → assemble classifier jars → Java tests on all 3 OSes → publish on tag |
| Perf regression | CI runs `frcsim_bench` scenarios and fails if p50 regresses >10% against the stored baseline (same-runner comparison) |

---

## 4. Repository Layout

Items marked *(planned)* don't exist yet.

```
physics-sim/
├── CLAUDE.md, README.md, CHANGELOG.md
├── VERSION                         # single version source, read by CMake and Gradle
├── docs/                           # IMPLEMENTATION_PLAN.md, building.md, adr/, (planned) models/, perf/, guides/
├── native/
│   ├── CMakeLists.txt, CMakePresets.json, .clang-format
│   ├── cmake/                      # FrcsimPlatform, FrcsimCompilerOptions, FrcsimDependencies (pinned + SHA-256)
│   ├── include/frcsim/frcsim_c.h   # stable C ABI (the only public native header)
│   ├── src/
│   │   ├── world/        # World, JoltRuntime, layers, materials + contact listener, shapes, kinematic/driven bodies
│   │   ├── field/        # Field (static geometry, bounds), field JSON loader
│   │   ├── pieces/       # PieceTypeRegistry, PiecePool (SoA, recycling, state machine)
│   │   ├── util/         # errors, lock-free EventBuffer
│   │   ├── capi/         # C ABI implementation (noexcept, status codes, thread-local last error)
│   │   ├── jni/          # JNI glue → C ABI only
│   │   ├── drive/        # (planned) DcMotor, Battery, SwerveModule, TireModel, SwerveVehicleController
│   │   ├── mechanisms/   # (planned) intake zones, indexer, flywheel shooter, aero
│   │   └── sensors/      # (planned) gyro, encoders
│   ├── tests/            # GoogleTest: core/ (links static core, replaces operator new), capi/ (shared lib only)
│   ├── bench/            # frcsim_bench scenario runner (-DFRCSIM_BUILD_BENCH=ON)
│   └── out/              # `cmake --install` output: <os>/<arch>/shared/ (git-ignored)
├── java/                           # Gradle 8.11 multi-project (wrapper)
│   ├── frcsim-core/      # org.frcsim: FrcSim, SimWorld, WorldConfig; org.frcsim.jni: NativeLoader, FrcSimJNI
│   ├── frcsim-native/    # packages native/out into WPILib JNI zips, generates the vendordep
│   ├── frcsim-jmh/       # JMH benchmarks of the binding (not published)
│   ├── frcsim-wpilib/    # (planned) WPILib adapters + telemetry
│   └── frcsim-games/     # (planned) season arenas (Rebuilt2026, ...)
├── vendordep/frcsim.json.in        # vendordep template (version, group, mavenUrl substituted)
├── examples/smoke-robot/           # stock WPILib 2026 project; end-to-end vendordep test
├── fields/test-flat/field.json      # REBUILT-sized flat field (schema: docs/reference/field-json.md)
├── tools/                          # (planned) calibration scripts
└── .github/workflows/ci.yml        # native matrix + ASan + Java + smoke robot (not yet run: no remote)
```

---

## 5. Runtime Architecture

### Layers
```
 Robot code (Java)  ──►  frcsim-wpilib (adapters, telemetry)  ──►  frcsim-core (handles, ByteBuffers)
                                                                           │  1 JNI call / period
                                                                           ▼
                                   capi (C ABI, noexcept)  ──►  World (C++)
                                                                  ├─ Drive (swerve controllers)
                                                                  ├─ Mechanisms (intake, shooter)
                                                                  ├─ Pieces (pool, aero, states)
                                                                  ├─ Field (static geo, sensors)
                                                                  └─ Jolt PhysicsSystem
```

### Per-robot-period pipeline (`world.step(0.020)`)
1. **Java** writes inputs into the shared input buffer: per-motor applied voltage (or onboard
   controller setpoints), mechanism commands, and shot requests.
2. **One JNI call** `step(dt, substeps)`.
3. **Native**, per substep (default **5 substeps = 4 ms**, configurable):
   1. Battery: sum currents from last substep → bus voltage (sag).
   2. Motors: current (with stator/supply limits) → torque.
   3. Shooter flywheels: integrate; fire queued shots (spawn/activate projectile bodies, subtract energy).
   4. Aero forces on the **airborne set only** (drag, Magnus, spin decay).
   5. Rolling resistance on the **grounded, awake** set.
   6. `PhysicsSystem::Update` (collision + constraints; swerve tire forces solved inside the vehicle
      constraint).
   7. Contact/sensor events → piece state transitions (landed, intaken, scored, out of bounds).
   8. Integrate encoder positions (double precision), gyro.
4. **Native** writes outputs into shared output buffers: robot poses, module rotor positions and
   velocities, currents, battery voltage, piece positions (float32 xyz, packed), piece count, event
   ring buffer (scores, intakes).
5. **Java** reads buffers (no allocation) → feeds WPILib/CTRE/REV sim states and telemetry.

### API sketch (Java, illustrative)
```java
var world = SimWorld.create(Rebuilt2026.arena());           // AutoCloseable; throws if RobotBase.isReal()
var robot = world.addSwerveRobot(SwerveConfig.builder()
    .mass(Kilograms.of(60)).moi(6.0).bumperSize(0.9, 0.9)
    .modules(ModuleConfig.mk4i(L2).driveMotor(DCMotor.getKrakenX60Foc(1)).steerMotor(...))
    .wheelCof(1.2).build(), startPose);
var shooter = robot.addFlywheelShooter(ShooterConfig...);
var intake  = robot.addIntakeZone(IntakeConfig.overBumper(...));

// simulationPeriodic():
robot.module(i).setDriveVoltage(v); robot.module(i).setSteerVoltage(v);
world.step(0.020);
talonSimState.setRawRotorPosition(robot.module(i).driveRotorRotations());
piecePublisher.set(world.pieces().positionsAsTranslation3d());  // reused array, no alloc
```

### C ABI shape (illustrative)
```c
typedef struct frcsim_world frcsim_world;
FRCSIM_API frcsim_status frcsim_world_create(const frcsim_world_config*, frcsim_world** out);
FRCSIM_API frcsim_status frcsim_world_step(frcsim_world*, double dt, int substeps);
FRCSIM_API frcsim_status frcsim_world_buffers(frcsim_world*, frcsim_buffers* out); /* stable pointers */
FRCSIM_API void          frcsim_world_destroy(frcsim_world*);
```
Configuration calls (add robot, load field) may be chatty, but they happen at startup only.

---

## 6. What to Model (research summary)

All units SI. Field frame = WPILib convention: origin at the blue alliance wall corner, +X toward red,
+Y left, +Z up, CCW positive. Values marked *(calibrate)* are starting guesses, to be replaced by
measurements (see §9 Phase 6).

### 6.1 Swerve drivetrain

**DC motor (per motor, quasi-static; inductance ignored because its electrical time constant ≪ 4 ms)**
```
I       = (V_applied − ω_rotor / Kv) / R        then clamp: stator limit, supply limit (I_supply ≈ I·|duty|)
τ_rotor = Kt · I
```
Take `R`, `Kv`, `Kt` from WPILib `DCMotor` factories (Kraken X60 / FOC, Falcon, NEO, Vortex) passed
from Java. Model the controller's brake vs. coast neutral mode (brake = shorted windings, damping
torque `Kt·ω/(Kv·R)`).

**Battery**: `V_bus = V_oc − R_internal · ΣI_supply`, R_internal ≈ 0.015–0.020 Ω + wiring *(calibrate)*,
clamp at brownout (6.75 V on roboRIO; make it configurable for Systemcore). Every motor on the robot
(including shooter and intake) draws from the same battery. That coupling produces realistic
acceleration loss during shooting.

**Gearbox + wheel inertia**
```
ω_rotor = G · ω_wheel
J_eff · ω̇_wheel = η · G · τ_rotor − r · F_long − τ_rolling_resist − b · ω_wheel
J_eff = J_wheel + G² · J_rotor          (reflected rotor inertia dominates; include it)
```
η (gearbox efficiency) ≈ 0.90–0.97 *(calibrate)*. Example: SDS MK4i L2 drive G = 6.75, steer G = 150/7,
wheel r = 0.0508 m.

**Tire–carpet contact (the core of realism)**
- Contact-point velocity `v_c` = chassis linear velocity + ω × r_module, rotated into the wheel frame.
- Longitudinal slip velocity `s_x = ω_wheel·r − v_c,x`; lateral slip velocity `s_y = −v_c,y`.
- Force from a saturating slip curve under a **combined friction circle**:
  `F = μ(|s|) · N · ŝ`, with `μ(s) = μ_peak · tanh(|s| / s₀)` (optionally a small post-peak drop to μ_kinetic).
  μ_peak ≈ 1.0–1.3 depending on tread *(calibrate)*; s₀ ≈ 0.02–0.1 m/s *(calibrate)*.
- **Stiffness warning:** the wheel-slip dynamics have a time constant around
  `J_eff / (r²·μN/s₀) ≈ 1 ms`, shorter than the 4 ms substep. **Explicit Euler on wheel spin will
  blow up.** Tire forces must be solved **implicitly, as velocity constraints with force limits**.
  Plan: implement a custom Jolt `VehicleController` (`SwerveVehicleController`, following the pattern of
  Jolt's `TrackedVehicleController` and `MotorcycleController`). Wheel steer comes from our steer
  dynamics, drive torque from our motor model, and longitudinal and lateral friction are solved as
  constraint impulses limited by the tire curve. *Phase 2 spike must confirm the Jolt vehicle API
  supports per-wheel steer and drive torque the way we need. If not, write a small implicit
  per-module solver and apply the resulting impulses to the chassis body.*
- **Normal force N per module** comes from Jolt wheel contact (suspension-as-constraint with very
  stiff spring + damping representing tread and carpet compliance). This automatically produces
  **load transfer** under acceleration (tall CoG → rear modules gain traction, tipping possible) and
  correct behavior on **bumps and ramps**.
- Rolling resistance: small `C_rr · N` opposing wheel rotation *(calibrate)*.
- Optional later: carpet nap directionality (μ varies with direction), a few percent.

**Steer axis**: steer motor + gearbox + module steer inertia (J ≈ 0.004–0.01 kg·m² at the module
*(calibrate)*) + **scrub torque** from the contact patch when steering at low speed. This makes steer
response realistic in place versus while moving.

**Chassis**: 6-DOF rigid body (mass incl. battery & bumpers, CoG height, inertia tensor). Collision
shape = compound: bumper box (restitution ≈ 0.08, friction ≈ 0.65 per maple-sim's tuned values),
frame, and optional mechanism shapes (extended intake, elevator) toggled at runtime.

**Sensors** (all optional noise, off by default for deterministic tests)
- Gyro (Pigeon 2 / NavX-like): yaw from chassis body + white noise + slow bias drift; pitch/roll for
  bumps and tipping detection.
- Encoders: rotor position quantization; drive encoder measures *wheel* rotation, so **wheel slip
  produces real odometry drift**. That drift is what makes the sim useful for testing pose estimators.
- CAN latency / timestamps for Phoenix 6 high-frequency odometry (250 Hz): expose per-substep samples
  so a sim odometry thread can consume them.

**Validation tests (analytic, GoogleTest)**
- Free speed on flat ground ≈ `(V − I_free·R)·Kv / G · r` minus rolling and efficiency losses.
- Max linear acceleration ≈ `min(μ·g, motor-limited)`; wheelspin appears when commanded torque
  exceeds the traction limit.
- Pushing match: two identical robots at full voltage, head-on → near-zero net motion.
- Current draw at stall matches `V_bus / R` under limits; battery sag matches formula.
- Rotation in place: steady-state ω matches kinematics.

### 6.2 Game pieces (reference: 2026 REBUILT fuel)

| Property | Value | Source / note |
|---|---|---|
| Shape | Sphere, high-density foam | Game manual |
| Diameter | 5.91 in = **0.150 m** | Manual / AndyMark |
| Mass | 0.448–0.500 lb = **0.203–0.227 kg** | Manual / AndyMark (support per-piece variance) |
| Count on field | **504** | Manual |
| Inertia | `(2/5)·m·r²` (solid foam sphere) | |
| Restitution vs carpet / ball / polycarbonate | ~0.4–0.6 / ~0.5 / ~0.6 | *(calibrate: drop test, high-speed video)* |
| Friction (ball-ball, ball-carpet) | ~0.6–0.9 (foam is grippy) | *(calibrate: incline test)* |
| Rolling resistance on carpet | significant (carpet absorbs); model as torque + linear damping when grounded | *(calibrate: roll-out distance)* |

**Piece state machine (native, SoA registry `PiecePool`)**
`ON_FIELD` (awake or sleeping Jolt body) → `IN_ROBOT` (body **deactivated and removed from
broadphase**, tracked as a count per robot slot) → `AIRBORNE` (active body, aero enabled, CCD enabled)
→ `ON_FIELD` | `SCORED` (sensor volume hit → hold, count, re-release per game rules) |
`OUT_OF_BOUNDS`.

- **Sleeping is essential.** Most of the 504 pieces sit still most of the time. Tune Jolt sleep
  thresholds so a resting ball on carpet sleeps within about 0.5 s.
- **Robot storage level of detail:** default is *abstract* (pieces inside a robot are a count plus an
  ordered queue, no physics). Optional *physical hopper* mode simulates pieces inside robot geometry;
  it's expensive and off by default.
- Batch add/remove bodies via Jolt `AddBodiesPrepare/Finalize`; never allocate at runtime. The pool is
  preallocated at world creation (capacity configurable, default 600).
- Game-agnostic: piece types are data (shape sphere/cylinder/box/convex hull, mass, material, aero
  params). Other seasons' pieces (e.g., notes, coral, algae) are configs, not new code.

### 6.3 Projectiles and aerodynamics

Airborne pieces are **real Jolt bodies** (not kinematic), so they bounce off the hub, rims, and robots.
Extra forces are applied each substep:
```
Re     = ρ·v·D/μ_air ≈ 1.0e5 at 10 m/s (D = 0.15 m)
F_drag = −½ · ρ · C_d · A · |v| · v                A = π r² = 0.01767 m²
F_lift = ½ · ρ · C_L(S) · A · |v|² · (ω̂ × v̂)      S = r·|ω| / |v|   (spin parameter)
τ_spin = −k_spin · ω · |ω|                          (spin decay; minor over ~1–1.5 s flights)
```
- ρ = 1.20 kg/m³ default, configurable (high-altitude events ≈ 1.0).
- C_d ≈ 0.47–0.5 for a sphere at this Re *(calibrate: foam surface roughness may shift the drag crisis)*.
- **Magnitude check:** at 10 m/s, drag ≈ 0.53 N versus weight ≈ 2.1 N (**~25% of gravity**), and
  terminal velocity is only about 20 m/s. **Drag cannot be ignored.**
- Typical hooded single-flywheel shots give S ≈ 1, beyond the linear `C_L ≈ S` regime (valid for
  S < 0.4). Use a **saturating table/curve for C_L(S)**: default ~0.2–0.35 for S ∈ [0.5, 1.5]
  *(calibrate)*. At S = 1, lift ≈ 15% of weight, so backspin visibly raises the arc.
- **CCD** (`EMotionQuality::LinearCast`) only for airborne pieces. A 15 m/s ball moves 6 cm per 4 ms
  substep, which can tunnel through thin rims and nets. Switch back to discrete collision on landing.
- Solve launch parameters with the *same* aero model in a Java utility (`ShotCalculator`), so teams can
  generate shooter lookup tables that match the sim (and compare them to reality).

### 6.4 Mechanisms

**Flywheel shooter**
```
J_fw · ω̇ = N_motors · G · τ_motor − b·ω − τ_friction
Shot: v_exit = η_grip · r_fw · ω_fw   (η_grip ~0.5–0.9 depends on compression, hood; calibrate)
      ω_ball from surface-speed difference (hood = 0 m/s, or top/bottom roller speeds)
      ΔE = ½ m v_exit² + ½ I_ball ω_ball²  →  ω_fw ← sqrt(ω_fw² − 2ΔE / (η_transfer · J_fw))
      + configurable Gaussian dispersion (speed %, yaw/pitch deg), seeded RNG
```
This captures **RPM dip under rapid fire**, the main realism gap in current sims. The shot exit pose
comes from robot pose ⊕ turret yaw ⊕ hood pitch, and the robot's chassis velocity is added to the
ball's exit velocity (shoot-on-the-move).

**Intake**
- Default: sensor-volume model (Jolt sensor body attached to the chassis) with acceptance conditions:
  roller powered, piece inside zone for ≥ t_min, relative approach speed within limits, capacity not
  full, custom predicate from Java. More realistic than instant "touch it, own it."
- Optional physical model: roller surfaces as contact surfaces with **surface velocity** (Jolt contact
  listener sets relative surface velocity, conveyor-belt style), so pieces get dragged in or bounced
  off.

**Indexer / feeder**: abstract queue with transit time per piece (default); feeds the shooter at a
rate limited by the feeder motor speed.

### 6.5 Field

- **Data-driven** definition in `fields/<season>/field.json`: static primitives (boxes, cylinders,
  convex hulls, heightfield/mesh for bumps and ramps), each with a material, plus sensor volumes
  (goals, zones) and initial piece placements. Hand-author simplified collision geometry. **Never use
  CAD meshes directly** (too many triangles).
- Borders/walls: boxes with polycarbonate or diamond-plate materials.
- REBUILT reference arena: hubs (funnel as convex pieces + goal sensor), bumps (drivable 3D
  geometry), trenches (low-clearance overhead), tower, depot/outpost piece sources. Take exact
  dimensions from the official field drawings and manual and verify them. Field is ~16.5 m × 8.1 m;
  use exact values from the drawings.
- Season rules (scoring, hub active/inactive windows, human-player re-entry of scored fuel) live in
  Java `frcsim-games`, driven by native events.
- Mirroring helpers for alliance-relative coordinates.

### 6.6 Intentionally not modeled (fidelity cutoffs)
Motor inductance and thermal effects (add thermal later if requested), gear backlash, wheel wear,
carpet seams, foam ball deformation beyond contact compliance, air currents, ball-to-ball aerodynamic
interaction, CAN bus saturation.

---

## 7. Performance Design

### Budgets (scenario runner, ~2020 4-core laptop, release build, default 2 physics worker threads)

Current measurements and history: [`docs/perf/phase1.md`](docs/perf/phase1.md). Single-threaded dense piles
measured 2.77 ms, which is why the default is 2 workers (D24).
| Scenario | Target per 20 ms robot period (5 substeps) |
|---|---|
| 504 fuel **sleeping** + 1 swerve robot driving | **≤ 0.5 ms** |
| 360 fuel **awake/colliding** + 2 robots plowing through them | **≤ 2.0 ms** |
| 504 fuel awake + 60 airborne (full-rate shooting) + 6 robots | **≤ 4.0 ms** |
| JNI overhead per `step()` incl. buffer reads | **≤ 50 µs** |
| Java allocations per period in steady state | **0** |
| Telemetry publish of 504 `Translation3d` | ≤ 0.3 ms (keep 3D poses optional; spheres need no rotation) |

Anything over 20 ms/period is a failure: the sim must stay real time with a 5× margin in the worst
scenario.

### Rules
- **No heap allocation inside `step()`** (native or Java). Preallocate pools at world creation; use
  Jolt's `TempAllocatorImpl` sized at startup.
- **Data-oriented design:** piece registry is SoA (`std::vector<float> px, py, pz` / state bytes / body
  IDs), not arrays of objects. The airborne set and grounded-awake set are compact index lists, so
  aero and rolling loops touch only relevant pieces.
- **No virtual dispatch in per-piece or per-contact hot loops.** Virtuals are fine per robot and per
  mechanism.
- Contact listener callbacks must be cheap and thread-safe: push compact event structs into a
  lock-free/preallocated buffer and process after `Update`.
- **Measure threading, don't assume.** Jolt's `JobSystemThreadPool` versus single-threaded at this
  scale; the default is whichever wins on the benchmarks (likely 2–4 threads, max).
- Output buffers are written in one linear pass; positions as float32, odometry and encoder
  accumulators as float64.
- JNI: direct `ByteBuffer`s with native byte order, created once; no `Get*ArrayElements` copies in the
  hot path; no JNI object creation per step. Handles are `long` wrapped in `AutoCloseable` with a
  `Cleaner` safety net.
- Every optimization PR includes before/after numbers from `frcsim_bench`.

---

## 8. Conventions

### General
- SI units everywhere internally and across the C ABI (meters, kg, seconds, radians, volts, amps).
  Java adapters convert to and from WPILib `Units` at the edge.
- Deterministic by default: fixed timestep, seeded RNG, no wall-clock reads inside the sim.
- Keep files focused (aim < 500 lines); one concept per file.
- Every physical constant that is a guess is marked `// CALIBRATE:` with its source.

### C++
- `namespace frcsim`, `snake_case` files, `PascalCase` types, `camelCase` functions, `m_` members
  (matches Jolt/WPILib style).
- No exceptions or RTTI across the ABI; C ABI functions are `noexcept` and return `frcsim_status`.
  Public C header is pure C; all other headers are private.
- Hide all symbols by default (`-fvisibility=hidden`); export only the C ABI and JNI entry points.
- `const`-correct, `[[nodiscard]]` on status-returning functions, no raw `new`/`delete` outside pools.

### Java
- `frcsim-core` must never import `edu.wpi.first.*` / `org.wpilib.*`.
- `SimWorld.create()` throws on a real robot (`RobotBase.isReal()` check lives in `frcsim-wpilib`).
- Builders for configs; configs immutable; runtime objects `AutoCloseable`.
- No allocation in methods documented as "periodic-safe".

### JNI
- JNI glue calls **only** the C ABI (never C++ internals), so a future Java FFM binding (Java 22+,
  WPILib 2027 = Java 25) can reuse the exact same surface.
- Convert native errors into Java exceptions with messages; never crash the JVM on bad input. Validate
  configs at creation time.

---

## 9. Roadmap

Each phase has an exit gate. Don't start the next phase until the gate passes.

**Phase 0 — Prove the pipeline (de-risk packaging first)**
CMake + Jolt builds on all 3 OSes; a trivial `frcsim_version()` goes through JNI; Gradle packages
classifier jars; a vendordep JSON installs into a **fresh WPILib 2026 project** and `simulateJava`
loads the native lib on Windows/macOS/Linux; robot deploy to a real controller is unaffected.
*Gate: CI green on the matrix, and the example robot project runs in sim on all 3 OSes.*

**Phase 1 — World + game pieces at scale**
World/stepping pipeline, field loader (flat carpet + walls), `PiecePool`, sleeping, piece state
machine, output buffers, `frcsim_bench` with the scenarios in §7, Tracy hooks.
*Gate: the 504-sleeping and 360-awake budgets in §7 are met (with a kinematic plow body standing in
for robots).*

**Phase 2 — Swerve drivetrain**
Spike: Jolt `VehicleController` suitability (decide constraint-based vs. custom implicit module
solver). Then DcMotor, Battery, SwerveModule, tire model, steer dynamics, chassis compound shape,
robot-robot collisions, sensors. Java `SwerveConfig` + WPILib/CTRE/REV adapters.
*Gate: all analytic validation tests in §6.1 pass; stable at 1–10 substeps; no NaNs under 10k-step
fuzz of random voltages and collisions.*

**Phase 3 — Field elements and REBUILT arena**
3D static geometry (bumps, hubs, trench, tower), sensor volumes, materials, `frcsim-games` Rebuilt2026
with scoring rules, AdvantageScope telemetry, example AdvantageKit swerve project.
*Gate: a robot drives over bumps, fuel scatters realistically, and budgets still hold.*

**Phase 4 — Shooting and mechanisms**
Aero (drag, Magnus, spin decay), CCD for airborne pieces, flywheel shooter with energy transfer and
dispersion, intake zones (sensor + optional surface-velocity rollers), indexer queue, `ShotCalculator`.
*Gate: 60 simultaneous airborne pieces within budget; the drag-free trajectory matches the analytic
solution exactly; the drag trajectory matches an independent RK4 reference within 1 cm.*

**Phase 5 — API hardening and docs**
API review, Javadoc, getting-started guide, examples (basic swerve, AKit + Phoenix 6 swerve, shooter,
multi-robot). Onboard motor-controller closed-loop models (velocity/position PID + FF at substep rate)
for teams not using vendor sim. CI job against the WPILib 2027 beta (Java 25), and an evaluation of an
FFM binding.

**Phase 6 — Calibration and validation against reality**
`tools/` scripts: fit tire μ and s₀ and gearbox efficiency from real AdvantageKit logs (acceleration
and wheelspin), drop and roll tests for fuel restitution and rolling resistance, fit C_d and C_L from
high-speed video of shots. Publish a "sim vs real" comparison report. Update all `CALIBRATE:` values.

---

## 10. Commands

Full details and other platforms: [`docs/building.md`](docs/building.md). All commands need `JAVA_HOME`
set to a JDK 17 (Windows: `C:\Users\Public\wpilib\2026\jdk`). On this dev machine CMake lives at
`C:\Program Files\CMake\bin` if it's not on `PATH`.

```bash
# native (from native/) - Windows shown; Linux/macOS use the ninja-release preset
cmake --preset windows-msvc
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
cmake --install build/windows-msvc --config Release    # -> native/out/windows/x86-64/shared/

# java (from java/) - requires the native install step above
./gradlew spotlessApply build                          # format, compile, javadoc, JUnit (+ JNI tests)
./gradlew :frcsim-native:installSmokeRobotVendordep    # publish to java/build/repo + install vendordep

# end-to-end (from examples/smoke-robot/)
./gradlew test                                          # stock WPILib project loads frcsim via vendordep
```
Performance (record results in `docs/perf/`):
```bash
# native scenario runner (from native/)
cmake --preset windows-msvc -DFRCSIM_BUILD_BENCH=ON
cmake --build --preset windows-msvc-release --target frcsim_bench
build/windows-msvc/bin/Release/frcsim_bench.exe --threads 2          # --help lists scenarios and flags

# Java binding microbenchmarks (from java/)
./gradlew :frcsim-jmh:jmh
```

---

## 11. Open Questions and Risks
- **Jolt vehicle API fit** for independent per-wheel steer and drive (Phase 2 spike). Fallback: custom
  implicit module solver applying impulses.
- **SIMD baseline:** SSE4.2-only may cost performance. Decide AVX2 dual build after Phase 1 numbers.
- **WPILib 2027 migration** (Java 25, package renames, Systemcore brownout behavior). Mitigated by the
  WPILib-free core and a thin adapter module.
- **Fuel material data is unmeasured.** All contact and aero params start as estimates until Phase 6.
- **Physical hopper mode** may blow the budget. It stays optional.
- **Name `frcsim`** is a placeholder; check for conflicts before first publish.

## 12. References
- maple-sim (architecture, known limitations): https://github.com/Shenzhen-Robotics-Alliance/maple-sim
- maple-sim fuel performance reports: https://www.chiefdelphi.com/t/maple-sim-alpha-bringing-robot-simulations-to-the-next-level-with-physics-engines/473238
- FuelSim (community fuel mini-library): https://www.chiefdelphi.com/t/introducing-fuelsim-a-simple-fuel-simulation-mini-library/512549
- 2026 REBUILT game manual: https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
- REBUILT fuel specs: https://andymark.com/products/official-rebuilt-fuel
- Jolt Physics: https://github.com/jrouwe/JoltPhysics
- WPILib vendor template (JNI/vendordep structure): https://github.com/wpilibsuite/vendor-template
- WPILib 2026 changes: https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
- WPILib 2027 (Java 25 / C++23, Systemcore): https://github.com/wpilibsuite/allwpilib/releases
- Lift and drag on spinning balls (C_L vs spin parameter): https://www.physics.usyd.edu.au/~cross/PUBLICATIONS/57.%20LiftDrag.pdf
- Baseball lift/drag vs spin: https://baseball.physics.illinois.edu/LyuDragLiftSpin.pdf
