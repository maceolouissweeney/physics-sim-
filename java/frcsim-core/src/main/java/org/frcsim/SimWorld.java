package org.frcsim;

import java.lang.ref.Cleaner;
import java.util.Objects;
import org.frcsim.jni.FrcSimJNI;
import org.frcsim.jni.NativeLoader;

/**
 * A physics world: field, game pieces, and (from Phase 2) robots.
 *
 * <p>Owns native memory. Close it (or use try-with-resources) when done; a {@link Cleaner} frees it
 * if the object becomes unreachable, but relying on that delays release. All views returned by this
 * world ({@link #pieces()}, {@link #stats()}, ...) throw {@link IllegalStateException} after close.
 *
 * <p>Not thread-safe. Drive a world from a single thread, typically {@code simulationPeriodic()}.
 */
public final class SimWorld implements AutoCloseable {
  private static final Cleaner CLEANER = Cleaner.create();

  private final WorldConfig config;
  private final NativeHandle handle;
  private final Cleaner.Cleanable cleanable;
  private final WorldStats stats;
  private final Materials materials;
  private final Field field;
  private final GamePieces pieces;
  private final KinematicBodies kinematics;

  private SimWorld(WorldConfig config, long nativePointer) {
    this.config = config;
    this.handle = new NativeHandle(nativePointer);
    this.cleanable = CLEANER.register(this, handle);
    try {
      this.stats = new WorldStats(this, FrcSimJNI.worldStatsBuffer(nativePointer));
      this.materials = new Materials(this);
      this.field = new Field(this);
      this.pieces =
          new GamePieces(
              this,
              FrcSimJNI.piecePositionsBuffer(nativePointer),
              FrcSimJNI.pieceStatesBuffer(nativePointer),
              config.maxPieces());
      this.kinematics = new KinematicBodies(this);
    } catch (RuntimeException e) {
      cleanable.clean();
      throw e;
    }
  }

  /**
   * Creates a world with {@link WorldConfig#defaults()}.
   *
   * @return new world
   */
  public static SimWorld create() {
    return create(WorldConfig.defaults());
  }

  /**
   * Creates a world.
   *
   * @param config world configuration
   * @return new world
   * @throws FrcSimException if the native library cannot be loaded or world creation fails
   */
  public static SimWorld create(WorldConfig config) {
    Objects.requireNonNull(config, "config");
    NativeLoader.ensureLoaded();
    long pointer =
        FrcSimJNI.worldCreate(
            config.maxBodies(),
            config.maxBodyPairs(),
            config.maxContactConstraints(),
            config.workerThreads(),
            config.tempAllocatorBytes(),
            -config.gravity(),
            config.maxPieces(),
            (float) config.minVelocityForRestitution(),
            (float) config.timeBeforeSleep(),
            (float) config.sleepVelocityThreshold(),
            config.solverVelocitySteps(),
            config.solverPositionSteps());
    return new SimWorld(config, pointer);
  }

  /**
   * Advances the simulation by {@code dtSeconds} using the configured number of sub-steps. Does not
   * allocate.
   *
   * @param dtSeconds time to advance, typically 0.020
   */
  public void step(double dtSeconds) {
    step(dtSeconds, config.substeps());
  }

  /**
   * Advances the simulation by {@code dtSeconds} using {@code substeps} equal fixed steps. Does not
   * allocate.
   *
   * @param dtSeconds time to advance, must be finite and &gt; 0
   * @param substeps number of physics steps, 1..{@value WorldConfig#MAX_SUBSTEPS}
   * @throws IllegalArgumentException for invalid arguments
   * @throws IllegalStateException if the world has been closed
   */
  public void step(double dtSeconds, int substeps) {
    FrcSimJNI.worldStep(nativeHandle(), dtSeconds, substeps);
  }

  /**
   * Returns simulated time since creation. Equivalent to {@code stats().timeSeconds()}.
   *
   * @return simulated time in seconds
   */
  public double timeSeconds() {
    return stats.timeSeconds();
  }

  /**
   * Rebuilds the broadphase acceleration structures. Call after adding many bodies mid-simulation;
   * loading a field and the first step do this automatically.
   */
  public void optimizeBroadPhase() {
    FrcSimJNI.worldOptimizeBroadPhase(nativeHandle());
  }

  /**
   * Live simulation statistics (zero-copy view over native memory).
   *
   * @return statistics view
   */
  public WorldStats stats() {
    return stats;
  }

  /**
   * Materials registry.
   *
   * @return materials
   */
  public Materials materials() {
    return materials;
  }

  /**
   * Static field geometry.
   *
   * @return field
   */
  public Field field() {
    return field;
  }

  /**
   * Game pieces.
   *
   * @return game pieces
   */
  public GamePieces pieces() {
    return pieces;
  }

  /**
   * Scripted kinematic bodies.
   *
   * @return kinematic bodies
   */
  public KinematicBodies kinematics() {
    return kinematics;
  }

  /**
   * Returns the configuration this world was created with.
   *
   * @return world configuration
   */
  public WorldConfig config() {
    return config;
  }

  /**
   * Returns whether {@link #close()} has been called.
   *
   * @return true if closed
   */
  public boolean isClosed() {
    return handle.isReleased();
  }

  /** Frees native memory. Idempotent. */
  @Override
  public void close() {
    cleanable.clean();
  }

  /** Native pointer; throws if closed. */
  long nativeHandle() {
    return handle.pointerOrThrow();
  }

  /** Throws if closed. Views call this before touching shared native memory. */
  void checkOpen() {
    handle.pointerOrThrow();
  }

  /** Cleaner action; must not reference the owning SimWorld. */
  private static final class NativeHandle implements Runnable {
    private volatile long pointer;

    NativeHandle(long pointer) {
      this.pointer = pointer;
    }

    long pointerOrThrow() {
      long p = pointer;
      if (p == 0) {
        throw new IllegalStateException("SimWorld has been closed");
      }
      return p;
    }

    boolean isReleased() {
      return pointer == 0;
    }

    @Override
    public void run() {
      long p = pointer;
      pointer = 0;
      if (p != 0) {
        FrcSimJNI.worldDestroy(p);
      }
    }
  }
}
