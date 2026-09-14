# ADR-0003: Swerve drivetrain as a custom Jolt `VehicleController`

- **Status:** Accepted
- **Date:** 2026-09-12
- **Context task:** P2.1 (spike)

## Context

Swerve realism depends on the tire–carpet contact. With a 4 ms substep, wheel slip dynamics settle in
about 1 ms (reflected rotor inertia vs tire force slope, CLAUDE.md §6.1). So tire forces **must not** be
integrated explicitly. Jolt ships a vehicle system (`VehicleConstraint` + `VehicleController`) built for
cars and tanks. The spike read Jolt v5.6.0's `VehicleConstraint.cpp`, `Wheel.cpp`,
`WheeledVehicleController.cpp`, and `TrackedVehicleController.*` to see whether swerve fits.

## Findings

Per physics substep, `VehicleConstraint::OnStep` (a `PhysicsStepListener`) runs:
1. `controller->PreCollide(dt)`
2. **Wheel ground probes** (ray / sphere / cylinder cast along the suspension direction) → contact point,
   normal, contact body. Contact axes come from `GetWheelLocalBasis`, which applies each wheel's
   **`mSteerAngle`** about its steering axis. **Per-wheel steering is supported directly.**
3. `controller->PostCollide(dt)`

During the solver's velocity iterations, `VehicleConstraint::SolveVelocityConstraint` solves the suspension
(a soft constraint, stable for stiff springs), then calls `controller->SolveLongitudinalAndLateralConstraints(dt)`.
There the controller calls `Wheel::SolveLongitudinalConstraintPart(min, max)` /
`SolveLateralConstraintPart(min, max)`. These are velocity constraints between the chassis and the
contact body, with **impulse bounds the controller chooses**.

`WheeledVehicleController` already solves the stiff wheel–ground coupling **implicitly**. Each iteration it
computes the linear impulse that would zero contact slip through the wheel's inertia,
`Δλ = (ω − v_rel/r) · I / r`, clamps the accumulated impulse to the friction limit
(`μ · suspension impulse`), applies it, then feeds the reaction back into the wheel's spin
`ω −= Δλ · r / I`. Engine torque is applied in `PostCollide` with an implicit (Gaussian elimination) step
that uses the previous step's ground impulse as an estimate. Longitudinal impulses are intentionally
not warm-started.

The vehicle's up/forward axes are configurable (`VehicleConstraintSettings::mUp/mForward`), so Z-up works.
Controllers, wheels, and settings are subclassable (`TrackedVehicleController` and `MotorcycleController` are
existing examples). Wheels are constructed in order by `VehicleController::ConstructWheel`.

## Decision

Implement `SwerveVehicleController : JPH::VehicleController`:

| Hook | Swerve behavior |
|---|---|
| `PreCollide` | Integrate each **steer axis** (DC motor + gearbox + module inertia, implicit back-EMF damping, Coulomb friction + contact scrub), then `wheel->SetSteerAngle(θ)` so this step's contact axes use the new angle. Integrate encoder positions. |
| `PostCollide` | Integrate each **drive motor** implicitly against the wheel + reflected rotor inertia, with the previous ground impulse as the load estimate (same scheme as Jolt). Apply stator/supply current limits and brake/coast. Sum supply currents into the robot **battery** model (sag and brownout take effect the next substep). |
| `SolveLongitudinalAndLateralConstraints` | Tire model: μ(slip speed) Stribeck-style curve × ground material factor. Longitudinal impulse as Jolt does (implicit wheel spin coupling). Lateral impulse bounded by the **remaining friction circle** `sqrt((μN)² − λ_long²)`, so drive force has priority and hard acceleration costs lateral grip. |
| `AllowSleep` | `false`: robots never sleep. |

Supporting choices:
- **Chassis:** one dynamic body. The collision shape is the bumper box raised above the carpet, with
  center of mass at the configured height (`OffsetCenterOfMassShape`) and explicit box inertia.
- **Suspension:** very short (1 cm travel), stiff (15 Hz, ζ = 0.7). It stands in for tread and carpet compliance
  and gives per-module **normal forces**, and therefore load transfer, from the suspension impulses.
- **Wheel probes:** `VehicleCollisionTesterCastCylinder` on a query-only object layer (`kWheelProbe`) that hits
  **static field geometry only**. Wheels don't ride on game pieces; bumpers push pieces instead.
- **No serialization:** controller settings are not serializable (no `ObjectStream` macros outside Jolt).

## Consequences

- ✅ Stable at 1–10 substeps without hand-written stiff integrators for the tire.
- ✅ Bumps, ramps, load transfer, and tipping come from the rigid-body solver for free.
- ✅ Robots interact with pieces, walls, and other robots through normal Jolt contacts (pushing matches
  are limited by the tire friction bounds).
- ❌ Motor back-EMF damping is implicit only within `PostCollide`, not inside solver iterations. At extreme
  slip the drive torque lags by one substep, which is acceptable at 4 ms.
- ❌ Tied to Jolt's vehicle internals (`Wheel` API). Contained in `drive/swerve_controller.*`.

## Alternatives rejected
- **Explicit per-module tire forces via `AddForce`:** unstable at 4 ms without sub-substepping (see stiffness
  estimate).
- **Custom implicit module solver outside Jolt applying impulses:** duplicates what the vehicle constraint
  already provides, and wouldn't participate in the contact solver's iterations.
