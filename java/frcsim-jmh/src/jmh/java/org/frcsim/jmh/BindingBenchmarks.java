package org.frcsim.jmh;

import java.io.IOException;
import java.nio.file.Path;
import java.util.concurrent.TimeUnit;
import org.frcsim.FrcSim;
import org.frcsim.GamePieceType;
import org.frcsim.GamePieces;
import org.frcsim.SimWorld;
import org.frcsim.WorldConfig;
import org.frcsim.jni.FrcSimJNI;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.TearDown;

/**
 * Measures what the Java binding adds on top of native simulation time (CLAUDE.md §7: JNI overhead
 * per step must be ≤ 50 µs, and periodic reads must not allocate).
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.NANOSECONDS)
public class BindingBenchmarks {

  /** Raw cost of one JNI round trip (no work on the native side). */
  @Benchmark
  public int jniRoundTrip(LoadedLibrary library) {
    return FrcSimJNI.abiVersion();
  }

  /**
   * Full {@code step()} on a world with no bodies, single-threaded: Jolt returns almost
   * immediately, so this is an upper bound on per-step binding overhead.
   */
  @Benchmark
  public void stepEmptyWorld(EmptyWorld state) {
    state.world.step(0.020);
  }

  /** Reading three stats fields from shared memory. */
  @Benchmark
  public double readStats(PopulatedWorld state) {
    return state.world.stats().timeSeconds()
        + state.world.stats().activeBodies()
        + state.world.stats().piecesSimulated();
  }

  /** Bulk copy of 504 piece positions into a reused float[] (telemetry path). */
  @Benchmark
  public int copy504Positions(PopulatedWorld state) {
    return state.pieces.copyPositionsMeters(state.positionsMeters);
  }

  /** Per-piece accessor loop over 504 pieces. */
  @Benchmark
  public double loop504Accessors(PopulatedWorld state) {
    GamePieces pieces = state.pieces;
    double sum = 0.0;
    int n = pieces.count();
    for (int i = 0; i < n; i++) {
      sum += pieces.xMeters(i) + pieces.yMeters(i) + pieces.zMeters(i);
    }
    return sum;
  }

  /** Ensures the native library is loaded before measuring JNI calls. */
  @State(Scope.Benchmark)
  public static class LoadedLibrary {
    /** Loads the library. */
    @Setup
    public void setup() {
      FrcSim.ensureLoaded();
    }
  }

  /** A single-threaded world without bodies. */
  @State(Scope.Thread)
  public static class EmptyWorld {
    SimWorld world;

    /** Creates the world. */
    @Setup(Level.Trial)
    public void setup() {
      world = SimWorld.create(WorldConfig.builder().workerThreads(0).build());
    }

    /** Frees the world. */
    @TearDown(Level.Trial)
    public void tearDown() {
      world.close();
    }
  }

  /** The test field with 504 settled fuel. */
  @State(Scope.Thread)
  public static class PopulatedWorld {
    SimWorld world;
    GamePieces pieces;
    final float[] positionsMeters = new float[504 * 3];

    /**
     * Loads the field, spawns 504 fuel, and lets them settle.
     *
     * @throws IOException if the field file cannot be read
     */
    @Setup(Level.Trial)
    public void setup() throws IOException {
      world = SimWorld.create();
      world
          .field()
          .loadJson(
              Path.of(System.getProperty("frcsim.repoDir"), "fields", "test-flat", "field.json"));
      GamePieceType fuel = world.pieces().findType("fuel").orElseThrow();
      float[] positionsXyzMeters = new float[504 * 3];
      for (int i = 0; i < 504; i++) {
        positionsXyzMeters[3 * i] = 5.0f + 0.3f * (i % 24);
        positionsXyzMeters[3 * i + 1] = 2.0f + 0.3f * (i / 24);
        positionsXyzMeters[3 * i + 2] = 0.076f;
      }
      world.pieces().spawn(fuel, positionsXyzMeters);
      for (int i = 0; i < 100; i++) {
        world.step(0.020);
      }
      pieces = world.pieces();
    }

    /** Frees the world. */
    @TearDown(Level.Trial)
    public void tearDown() {
      world.close();
    }
  }
}
