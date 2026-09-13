# ADR-0001: C++ simulation core on Jolt Physics, exposed to Java via a batched C ABI + JNI

- **Status:** Accepted
- **Date:** 2026-09-12

## Context

FRC robot code is mostly Java. The simulation must handle 360+ moving game pieces (REBUILT puts 504 on
the field), realistic swerve traction, and projectile flight, all far faster than real time on
ordinary student laptops. The existing community library, maple-sim, uses dyn4j (2D, Java). Users report
1–2 fps when REBUILT fuel is disturbed, and its projectiles are kinematic with no drag.

## Decision

1. The simulation core is **C++20** built on **Jolt Physics** (v5.6.0, MIT).
2. The core is exposed through a **pure C ABI** (`frcsim_c.h`). Handles are opaque, errors are status
   codes plus a thread-local message, and config structs carry `struct_size`.
3. Java binds with **hand-written JNI** that calls only the C ABI. The binding crosses the boundary a
   **constant number of times per robot period** (target: one `step()`); bulk state moves through
   shared direct buffers (Phase 1).
4. The core has **no WPILib dependency**. WPILib adapters live in a separate Java module.

## Consequences

- ✅ SIMD, cache-friendly layout, no GC pauses or JIT warm-up in the hot loop.
- ✅ Jolt provides robust 3D collision, sleeping, CCD, sensors, contact surface velocities, and a
  subclassable vehicle system for the swerve tire model.
- ✅ The C ABI allows a Java FFM binding (WPILib 2027 uses Java 25), C++ robot code, or Python later.
- ✅ WPILib's yearly breaking changes touch only the thin adapter module.
- ❌ Native builds for Windows, Linux, and macOS (universal) are required. CI and packaging are more
  complex (mitigated by doing Phase 0 packaging first).
- ❌ Debugging crosses a language boundary. Mitigated by the no-exceptions-across-ABI rule and
  extensive C++ tests.
- ❌ Teams can't casually edit physics code in Java. Season rules and arenas stay in Java for that reason.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Pure Java custom solver | Viable fallback, but no dependable SIMD (Vector API still incubating), GC/JIT variance, and we'd write a 3D contact solver from scratch. |
| jolt-jni (existing bindings) | 1:1 fine-grained API means per-body JNI calls every tick. |
| dyn4j / JBox2D / Box2D v3 | 2D only: no ramps, stacking, or bouncing off goals. |
| PhysX | Heavy build and distribution; no advantage at this scale. |
| MuJoCo | Great for articulated contact; not built for hundreds of free bodies. |
