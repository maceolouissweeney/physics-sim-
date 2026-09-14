# Swerve drivetrain model

Reference for what `native/src/drive/` simulates, with equations and parameters. Design rationale:
[ADR-0003](../adr/0003-swerve-tire-solver.md). Values marked **CALIBRATE** are estimates pending Phase 6.

## Frames and conventions
- **Robot frame:** origin at the center of the frame footprint **on the carpet**, +X forward, +Y left, +Z up.
  Module positions are given in this frame.
- **Module angle:** 0 = wheel rolling toward robot +X, positive counterclockwise (same as WPILib
  `SwerveModuleState`). Reported **continuously** (not wrapped).
- **Drive encoder:** motor rotor position (rad), i.e. `wheel angle × gear ratio`. It measures wheel rotation,
  so **wheel slip shows up as odometry drift**.
- **Voltage sign:** positive drive voltage rolls the wheel forward along its module angle; positive steer
  voltage increases the module angle.

## DC motor (`drive/dc_motor.*`)
Constants from WPILib `DCMotor` parameters (nominal voltage `V_n`, stall torque, stall current, free current,
free speed) for `n` motors on one gearbox:
```
τ_stall = n · τ_stall,1      I_stall = n · I_stall,1      I_free = n · I_free,1
R  = V_n / I_stall
Kv = ω_free / (V_n − R · I_free)       (rad/s per volt)
Kt = τ_stall / I_stall                  (N·m per amp)
```
Presets: Kraken X60 (+FOC), Kraken X44 (+FOC), Falcon 500 (+FOC), NEO, NEO Vortex, taken from WPILib 2026.

**Applied voltage:** `V = clamp(V_cmd, −V_bus, V_bus)`. In **coast** mode with `V_cmd = 0` the windings are open
(`I = 0`); in **brake** mode they're shorted (`V = 0`, back-EMF braking).

**Geared mechanism step (implicit).** Mechanism speed ω (after the gearbox, ratio `G` = motor turns per
mechanism turn), efficiency η, mechanism-side inertia `J = J_mech + G² · n · J_rotor`, external torque `τ_ext`:
```
motor current   I = (V − G·ω/Kv) / R
mechanism torque τ = η · G · Kt · I = A − B·ω,   A = η·G·Kt·V/R,   B = η·G²·Kt/(Kv·R)
implicit Euler  ω' = (J·ω + dt·(A + τ_ext)) / (J + dt·B)
```
The back-EMF term `B` is stiff: the reflected rotor inertia makes `dt·B/J` about 0.8 at 4 ms for a Kraken L2
module. That's why the step is implicit.

**Current limits** (0 = disabled), applied after the unlimited solve. A current-limited controller lowers its
duty cycle, so the voltage it actually applies is the **effective voltage** `V_eff = I·R + G·ω/Kv`
(resistive drop + back-EMF), not the command:
- stator: `|I| ≤ I_stator`
- supply: `|I · V_eff / V_bus| ≤ I_supply` ⇒ largest `|I|` with `R·I² + e·I − I_supply·V_bus = 0`,
  where `e` is the back-EMF along the current's direction

If either binds, the torque is constant, `ω' = ω + dt·(η·G·Kt·I_lim + τ_ext)/J`, and `V_eff` is reported as
the applied voltage. Supply current: `I_supply = I · V_applied / V_bus`.

Example: a stalled Kraken X60 at an 80 A stator limit applies only 80·0.033 ≈ 2.6 V and draws ≈ 17.5 A from
the battery, not 80 A. An early version reported 80 A, and four modules browned out the battery during a
standing launch, which the traction validation test caught.

**Coulomb friction** `τ_f` (gearbox, bearings) is applied after the motor step without overshoot:
`ω' → 0` if `|ω'| ≤ τ_f·dt/J`, else `ω' −= sign(ω')·τ_f·dt/J`.

## Battery (`drive/battery.*`)
```
V_bus = max(0, V_oc − R_int · Σ I_supply)          (all robot motors; applied the next substep)
brownout: V_bus < V_brownout (6.75 V) → all motor outputs disabled (coast) until V_bus > V_recover (7.5 V)
```
Defaults: `V_oc` 12.5 V, `R_int` 0.020 Ω (battery + wiring). **CALIBRATE.**

## Steer axis
Mechanism = module rotating about its vertical axis. `J_mech` = module steer inertia (default 0.004 kg·m²,
**CALIBRATE**). Friction = steer friction torque + **contact scrub** `μ_s · N · r_scrub`, which resists
turning a loaded wheel in place (`r_scrub` ≈ 1 cm contact-patch lever arm, **CALIBRATE**). Integrated in
`PreCollide`; the resulting angle is applied to the Jolt wheel before this step's ground probes.

## Drive axis and tire (`drive/swerve_controller.*`, `drive/tire_model.h`)
Mechanism = wheel. `J = J_wheel + G²·n·J_rotor`. In `PostCollide` the drive motor is stepped with
`τ_ext = −λ_prev · r / dt` (last step's ground impulse, the same estimate Jolt uses). That estimate is
removed again because the solver applies the actual ground impulse.

In every solver velocity iteration, for each wheel in contact:
```
v_rel     = chassis point velocity − ground point velocity at the contact
slip      = | (ω·r − v_rel·t_long , v_rel·t_lat) |                  (m/s)
μ         = [μ_k + (μ_s − μ_k)·exp(−(slip/s₀)²)] · f_ground          (Stribeck-style; f_ground = ground material friction)
λ_max     = μ · λ_suspension                                         (normal impulse this step)
Δλ_long   = (ω − v_rel·t_long / r) · J / r                           (impulse that zeroes longitudinal slip)
λ_long    = clamp(λ_long,prev + Δλ_long, ±λ_max);   ω −= (λ_long − λ_long,prev) · r / J
λ_lat     ∈ ±sqrt(λ_max² − λ_long²)                                   (friction circle; drive has priority)
```
Defaults: `μ_s` 1.1, `μ_k` 0.9, `s₀` 0.1 m/s. **CALIBRATE** per tread (measure with a pull test).

## Chassis
- One rigid body: bumper box (`frameHalfX/Y`, from `bumperBottom` to `bumperBottom + bumperHeight` above
  the carpet), `mass`, center of mass at `(comX, comY, comHeight)`, box inertia (yaw inertia overridable).
- Bumper material default `bumper` (μ 0.65, e 0.08).
- Wheels: short stiff suspension (travel 1 cm, 15 Hz, ζ 0.7) along −Z from `(x, y, r + travel)`. The body
  origin rests about 0.3 mm below the carpet plane under static load.
- Wheel probes hit **static field geometry only** (`kWheelProbe` layer).

## Sensors (`SensorParams`, all ideal by default)
Robot outputs report **ground truth** (`yaw`, pose, velocities) and **measurements** separately:

| Measurement | Model |
|---|---|
| Gyro yaw (`gyro_yaw`) | `ψ_g = ψ₀ + (ψ − ψ₀)·(1 + scaleError) + driftRate·t + N(0, σ)`, where `ψ₀` and `t` reset with `resetPose` (like re-zeroing a Pigeon 2) |
| Drive encoder (`drive_rotor_position`) | rotor angle `wheelAngle·G`, floored to `2π / countsPerRev` when `countsPerRev > 0` |
| Module angle, velocities, currents | exact |

Noise comes from a seeded `std::mt19937` per robot: reproducible run to run on one platform, but not
identical across compilers (standard library normal distributions differ). Useful starting values for a
Pigeon 2: drift of a few ×10⁻⁵ rad/s and noise ≈ 0.001 rad (**CALIBRATE** against your robot).

Because the drive encoder measures wheel rotation, **wheel slip still produces odometry drift even with
ideal sensors**. That's intentional, not a sensor error.

## Defaults (SDS MK4i L2, Kraken X60 drive and steer, 60 kg)
| Parameter | Default |
|---|---|
| Wheel radius / width | 0.0508 m / 0.038 m |
| Wheel inertia | 3.0e-4 kg·m² |
| Drive ratio / efficiency | 6.75 / 0.95 |
| Steer ratio / efficiency | 150/7 / 0.90 |
| Rotor inertia (Kraken X60) | 6.0e-5 kg·m² (**CALIBRATE**) |
| Drive friction torque | 0.2 N·m at the wheel (**CALIBRATE**) |
| Steer friction torque | 0.3 N·m at the module (**CALIBRATE**) |
| Drive / steer stator limit | 80 A / 40 A |
| Robot mass, CoG height | 60 kg, 0.18 m |
