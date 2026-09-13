# Phase 1 performance report

- **Date:** 2026-09-12
- **Machine:** Intel Core i7-10610U (4 cores / 8 threads, 15 W laptop), Windows 11
- **Build:** MSVC 19.44 Release, SSE4.2 baseline (D8), Jolt Physics v5.6.0, static CRT
- **Tool:** `native/build/windows-msvc/bin/Release/frcsim_bench.exe` (source: `native/bench/scenario_runner.cpp`)
- **Metric:** wall-clock time per `World::step(0.020, 5)`, i.e. one 20 ms robot period with 5 physics substeps

Reproduce:
```
cmake --preset windows-msvc -DFRCSIM_BUILD_BENCH=ON
cmake --build --preset windows-msvc-release --target frcsim_bench
build/windows-msvc/bin/Release/frcsim_bench.exe --threads 2
```

## Scenarios

All use `fields/test-flat` (REBUILT-sized carpet with perimeter walls) and REBUILT fuel (0.150 m, 0.215 kg foam).
Robot stand-ins are **driven bodies**: 60 kg, 0.9 m square, drive force capped at 590 N (μ = 1).

| Scenario | Description | Budget (CLAUDE.md §7) |
|---|---|---|
| `empty_field` | 1 robot driving, no pieces (fixed-overhead baseline) | — |
| `sleeping_504` | 504 fuel resting (asleep) spread over mid-field; 1 robot driving in open carpet | ≤ 0.5 ms |
| `awake_360_plow` | 360 fuel packed 0.16 m apart; 2 robots driving back and forth through the pile | ≤ 2.0 ms |
| `full_504_plow` | Same with 504 fuel (stress test) | — |

## Results (defaults: 2 worker threads, 10 velocity / 2 position solver iterations)

| Scenario | p50 ms | p95 ms | p99 ms | Budget | Status |
|---|---|---|---|---|---|
| `empty_field` | 0.100 | 0.130 | 0.156 | — | |
| `sleeping_504` | 0.104 | 0.124 | 0.137 | 0.5 | ✅ |
| `awake_360_plow` | 1.45 | 2.80 | 3.48 | 2.0 | ✅ (p50) |
| `full_504_plow` | 1.99 | 4.13 | 4.57 | — | |

Worst case measured (`full_504_plow` p99 ≈ 4.6 ms) is still **4× faster than real time**.

### Thread count (p50 ms, 10/2 iterations)

| Workers | empty_field | sleeping_504 | awake_360_plow | full_504_plow |
|---|---|---|---|---|
| 0 (single-threaded) | 0.026 | 0.025 | 2.77 | 4.02 |
| 1 | 0.052 | 0.059 | ~3.0* | ~5.0* |
| **2** | 0.100 | 0.104 | **1.45** | **1.99** |
| 3 | — | — | 1.78 | 2.34 |
| 4 | — | — | 1.68 | 2.35 |

\* from an earlier run of the same build session; laptop run-to-run variance is about ±30% (turbo/thermal),
so only compare numbers from the same session.

### Solver iterations
Sweeping velocity steps {10, 6, 4} × position steps {2, 1} changed dense-pile p50 by under 5% at every
thread count. Collision detection, not the solver, dominates, so defaults stay (D25).

## Findings that changed the design

1. **Pieces need their own broadphase tree.** With pieces sharing Jolt's "moving" tree with the robot,
   every step rebuilt a 505-body tree: `sleeping_504` cost **0.50 ms**. A dedicated piece layer
   brought it to **0.025 ms** (single-threaded), a 20× improvement. See `world/layers.h`.
2. **Kinematic pushers are not robot proxies.** Infinite-mass plows squeezed piles against the
   carpet and **ejected 173 of 360 pieces** over the walls (exits up to z = 10 m). Force-limited driven
   bodies keep all pieces on the field and stall against dense piles like a traction-limited robot.
3. **Postprocessing skips sleeping pieces.** `PiecePool::postStep` reads positions only for awake bodies,
   using lock-free body access.
4. **Jolt allocates during broadphase rebuilds.** Measured 2 allocations per substep from
   `QuadTree::UpdatePrepare` (`new NodeID[numBodies]`). Accepted and bounded by a test (D22).
5. **Two worker threads are optimal** on a 4-core/15 W laptop (D24).

## Gaps and next candidates

- Single-threaded dense piles miss the budget (2.77 ms vs 2.0 ms). Mitigated by the 2-worker default.
- p95/p99 for `awake_360_plow` exceed 2 ms. Candidates, in expected-value order:
  1. **AVX2 build variant** selected at load time (revisit D8; Jolt's collision kernels are SIMD-heavy).
  2. Contact cache tuning (`mBodyPairCacheMaxDeltaPositionSq`, `mBodyPairCacheCosMaxDeltaRotationDiv2`) so
     slowly rolling pile pieces reuse manifolds.
  3. One `PhysicsSystem::Update(dt, collisionSteps = 5)` call instead of 5 single-step calls (less job
     setup per substep).
  4. Sleep tuning so pieces at the edge of a disturbed pile fall asleep sooner.
- Not yet measured: Linux/macOS numbers (CI).

## Java binding overhead (JMH)

`./gradlew :frcsim-jmh:jmh` (JMH 1.37, Temurin 17, 1 fork, 3 × 1 s warmup, 5 × 1 s measurement).
Source: `java/frcsim-jmh/src/jmh/java/org/frcsim/jmh/BindingBenchmarks.java`.

| Benchmark | Avg | Notes |
|---|---|---|
| `jniRoundTrip` | 28 ns | one JNI call with no native work |
| `readStats` | 22 ns | three fields from the shared stats block (no JNI) |
| `copy504Positions` | 216 ns | bulk copy of 504 xyz triples into a reused `float[]` (no JNI) |
| `loop504Accessors` | 14 µs (±34 µs, noisy) | `x/y/z(i)` for 504 pieces: per-call closed and index checks add up; prefer `copyPositions` for telemetry |
| `stepEmptyWorld` | 39 µs | full `step()` on a single-threaded world with no bodies: native Jolt work + one JNI call. Upper bound on binding overhead. |

The binding adds one ~28 ns JNI crossing per step. The **≤ 50 µs JNI overhead budget is met** by three orders
of magnitude, and periodic reads don't cross JNI at all.
