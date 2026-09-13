# World and game pieces (Phase 1 design)

How the native core organizes a simulation world, field geometry, materials, and game pieces.
Status: **implementing** (see `IMPLEMENTATION_PLAN.md` §4). Keep this document in sync with the code.

## Ownership

```
World
├── JoltRuntime            process-wide Jolt registration (refcounted)
├── MaterialTable          named materials + 64×64 pair table          world/materials.*
├── MaterialContactListener applies pair friction/restitution per contact world/contact_listener.*
├── Jolt PhysicsSystem (+ TempAllocator, JobSystem, layer filters)
├── Field                  static bodies (ground, walls, elements), bounds field/field.*
├── PieceTypeRegistry      piece shapes & mass properties                pieces/piece_type.*
├── PiecePool              SoA state for every piece, body recycling     pieces/piece_pool.*
├── KinematicBodies        scripted movers (tests, moving field elements) world/kinematic_bodies.*
└── DrivenBodies           force-limited robot stand-ins (step listener)  world/driven_bodies.*
```

## Collision layers

| Object layer | Broadphase tree | Collides with |
|---|---|---|
| `Static` | NonMoving | Robot, Piece |
| `Robot` (robots, kinematic, driven) | Robot | everything except Sensor↔Sensor |
| `Piece` | **Piece** | everything except Sensor↔Sensor |
| `Sensor` | Robot | Robot, Piece |

Pieces have their **own broadphase tree**. Jolt rebuilds a tree every step if anything in it moved, so
sharing a tree with always-moving robots rebuilt the 500-piece tree every step. Measured on an
i7-10610U: 504 sleeping pieces + 1 driving robot went from 0.50 ms to 0.025 ms per 20 ms period.

C++ member declaration order enforces lifetimes. The runtime and contact listener outlive the physics
system, and all body owners (field, pieces, kinematics) are destroyed before it.

## Body tags

Every body's Jolt user data (`uint64`) is a `BodyTag` (`world/body_tag.h`):

| bits | field | meaning |
|---|---|---|
| 63–60 | kind | `Field`, `Robot`, `Piece`, `Sensor`, `Kinematic` |
| 59–52 | material | index into `MaterialTable` |
| 51–32 | reserved | |
| 31–0 | index | index within the owner (piece index, field primitive index, …) |

Contact callbacks decode tags without locking or looking anything up.

## Materials

- A material has `friction` (≥ 0) and `restitution` (0..1). Id 0 is `default`.
- The combined value for a pair defaults to Jolt's rules: friction `sqrt(a·b)`, restitution `max(a, b)`.
  `setPair(a, b, …)` overrides a specific pair (symmetric).
- The pair table is dense (64×64), so the contact listener does two shifts and one array read.
- `registerStandardFrcMaterials()` provides starting values (`carpet`, `polycarbonate`, `aluminum`,
  `bumper`, `foam`, `foam↔carpet`). **All are CALIBRATE estimates** (see CLAUDE.md §6).
- Tables are mutable only between steps (contact callbacks read them from worker threads).

## Field

- Static primitives: ground slab, boxes, Z-axis cylinders, convex hulls. Each is one static body on
  `ObjectLayers::kStatic`, tagged `Field`.
- Statics use a 5 mm convex radius (D19) so edges stay sharp.
- `bounds` (AABB): pieces whose position leaves the bounds become `OutOfBounds` and are removed from
  the simulation.
- Loaded from JSON (`field/field_json.*`, schema in `docs/reference/field-json.md`) or built in C++.

## Game pieces

### Types
`PieceTypeDesc`: shape (sphere / Z-cylinder / box), dimensions, mass, material, max angular velocity.
The Jolt shape is built once per type and shared by all bodies of that type. Mass is set explicitly;
inertia comes from the shape scaled to that mass.

### Pool layout (structure of arrays)
Arrays are indexed by piece index (`0 … capacity-1`), all allocated at world creation:

| array | type | notes |
|---|---|---|
| `m_bodies` | `JPH::BodyID` | invalid until first spawn at that index |
| `m_typeOf` | `uint16` | fixed once a body exists (bodies are reused only for the same type) |
| `m_states` | `uint8` (`PieceState`) | output buffer, read directly by bindings |
| `m_positions` | `float32 × 3` | output buffer, refreshed after every `World::step` |
| `m_freeByType[t]` | `uint32` list | inactive indices whose body can be reused for type `t` |

Indices are **stable handles**: an index keeps its identity until despawned, then may be reused by a
later spawn of the same type. `highWater` is the number of indices ever used; output consumers read
`[0, highWater)`.

### State machine
```
            spawn                  setState(InRobot/Scored)
Inactive ───────────► OnField ◄──────────────────────────► InRobot / Scored
   ▲                  ▲    │ ▲                                (body removed from broadphase)
   │   despawn        │    │ └──── (Phase 4: launched) ────► Airborne
   └──────────────────┴────┘
                           └── leaves field bounds ──► OutOfBounds (removed)
```
- Physical states (`OnField`, `Airborne`) have their body in the broadphase; other states don't.
- Leaving a physical state removes the body (`RemoveBody`), and entering one re-adds it. The body is
  never destroyed (D17), so steady state has **zero allocations**.
- Batch spawns use `AddBodiesPrepare/Finalize`, the fast path for inserting many bodies at once.

### Per-step work
After all substeps, `PiecePool::postStep` runs one linear pass over `[0, highWater)`. For each **awake**
physical piece it reads the position into the output array (lock-free body access) and checks bounds.
Pieces that go out of bounds are removed right there. Sleeping pieces are skipped because their
outputs are already current: spawn and teleport write them directly.

## Sleeping and restitution
- Jolt puts a body to sleep after `timeBeforeSleep` (0.5 s) below `sleepVelocityThreshold` (0.03 m/s).
  Sleeping pieces cost nothing in collision/solver. This is how a full field of 504 fuel stays cheap.
- `minVelocityForRestitution` = 0.2 m/s (D15).

## Kinematic bodies
Scripted boxes with infinite mass (test obstacles, moving field elements). `moveTo(pose, dt)` sets a
velocity that reaches the target at the end of the next `step(dt)`, so the motion is correctly spread
over the substeps.

**Don't use kinematic bodies as robots.** An infinite-mass pusher squeezing a dense pile against the
carpet can't be resolved by the solver, so pieces get ejected at high speed. The first benchmark lost
173 of 360 pieces over the walls and up to z = 10 m.

## Driven bodies
Dynamic 60 kg-class boxes restricted to planar motion (`EAllowedDOFs::Plane2D`, no gravity) that track a
target velocity. A Jolt `PhysicsStepListener` applies, every substep:
```
F = clamp(m · (v_target − v) / h, |F| ≤ maxForce)        (maxForce ≈ μ·m·g, e.g. 590 N)
τ = clamp(I_z · (ω_target − ω) / h, |τ| ≤ maxTorque)
```
Because force is limited, a driven body stalls against a dense pile or a wall like a real robot does.
They are the robot stand-in for benchmarks until the swerve model lands (Phase 2). Native-only for now.

## Zero-allocation guarantee
`frcsim_core_tests` replaces global `operator new` and hooks Jolt's allocator. A test steps a
populated world with pieces being plowed and asserts **no allocations** happen during the measured
steps.
