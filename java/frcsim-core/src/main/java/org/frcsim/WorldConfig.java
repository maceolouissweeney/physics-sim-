package org.frcsim;

/**
 * Immutable configuration for a {@link SimWorld}. Build with {@link #builder()}.
 *
 * <p>Capacities are fixed at creation; the simulation never allocates while stepping.
 */
public final class WorldConfig {
  /** Standard gravity in m/s². */
  public static final double STANDARD_GRAVITY = 9.80665;

  /** Maximum number of worker threads accepted by the native library. */
  public static final int MAX_WORKER_THREADS = 64;

  /** Maximum sub-steps per step call accepted by the native library. */
  public static final int MAX_SUBSTEPS = 1000;

  /** Maximum solver iterations accepted by the native library. */
  public static final int MAX_SOLVER_STEPS = 100;

  private final int maxBodies;
  private final int maxBodyPairs;
  private final int maxContactConstraints;
  private final int workerThreads;
  private final int tempAllocatorBytes;
  private final double gravity;
  private final int substeps;
  private final int maxPieces;
  private final double minVelocityForRestitution;
  private final double timeBeforeSleep;
  private final double sleepVelocityThreshold;
  private final int solverVelocitySteps;
  private final int solverPositionSteps;

  private WorldConfig(Builder b) {
    requirePositive("maxBodies", b.maxBodies);
    requirePositive("maxBodyPairs", b.maxBodyPairs);
    requirePositive("maxContactConstraints", b.maxContactConstraints);
    if (b.workerThreads < -1 || b.workerThreads > MAX_WORKER_THREADS) {
      throw new IllegalArgumentException(
          "workerThreads must be -1 (auto), 0 (single-threaded), or 1.." + MAX_WORKER_THREADS);
    }
    if (b.tempAllocatorBytes < 1024 * 1024) {
      throw new IllegalArgumentException("tempAllocatorBytes must be >= 1 MiB");
    }
    requireNonNegative("gravity", b.gravity);
    if (b.substeps < 1 || b.substeps > MAX_SUBSTEPS) {
      throw new IllegalArgumentException("substeps must be in 1.." + MAX_SUBSTEPS);
    }
    if (b.maxPieces < 1 || b.maxPieces > b.maxBodies) {
      throw new IllegalArgumentException("maxPieces must be in 1..maxBodies");
    }
    requireNonNegative("minVelocityForRestitution", b.minVelocityForRestitution);
    requireNonNegative("timeBeforeSleep", b.timeBeforeSleep);
    requireNonNegative("sleepVelocityThreshold", b.sleepVelocityThreshold);
    if (b.solverVelocitySteps < 1 || b.solverVelocitySteps > MAX_SOLVER_STEPS) {
      throw new IllegalArgumentException("solverVelocitySteps must be in 1.." + MAX_SOLVER_STEPS);
    }
    if (b.solverPositionSteps < 0 || b.solverPositionSteps > MAX_SOLVER_STEPS) {
      throw new IllegalArgumentException("solverPositionSteps must be in 0.." + MAX_SOLVER_STEPS);
    }
    maxBodies = b.maxBodies;
    maxBodyPairs = b.maxBodyPairs;
    maxContactConstraints = b.maxContactConstraints;
    workerThreads = b.workerThreads;
    tempAllocatorBytes = b.tempAllocatorBytes;
    gravity = b.gravity;
    substeps = b.substeps;
    maxPieces = b.maxPieces;
    minVelocityForRestitution = b.minVelocityForRestitution;
    timeBeforeSleep = b.timeBeforeSleep;
    sleepVelocityThreshold = b.sleepVelocityThreshold;
    solverVelocitySteps = b.solverVelocitySteps;
    solverPositionSteps = b.solverPositionSteps;
  }

  /**
   * Returns the default configuration: 2 physics worker threads, standard gravity, 5 sub-steps (4
   * ms physics step at a 20 ms robot period), room for 1024 game pieces.
   *
   * @return default configuration
   */
  public static WorldConfig defaults() {
    return builder().build();
  }

  /**
   * Returns a builder initialized with defaults.
   *
   * @return new builder
   */
  public static Builder builder() {
    return new Builder();
  }

  /**
   * Returns a builder initialized with this configuration's values.
   *
   * @return new builder
   */
  public Builder toBuilder() {
    Builder b = new Builder();
    b.maxBodies = maxBodies;
    b.maxBodyPairs = maxBodyPairs;
    b.maxContactConstraints = maxContactConstraints;
    b.workerThreads = workerThreads;
    b.tempAllocatorBytes = tempAllocatorBytes;
    b.gravity = gravity;
    b.substeps = substeps;
    b.maxPieces = maxPieces;
    b.minVelocityForRestitution = minVelocityForRestitution;
    b.timeBeforeSleep = timeBeforeSleep;
    b.sleepVelocityThreshold = sleepVelocityThreshold;
    b.solverVelocitySteps = solverVelocitySteps;
    b.solverPositionSteps = solverPositionSteps;
    return b;
  }

  /**
   * Body capacity for all bodies (field, robots, pieces).
   *
   * @return body capacity
   */
  public int maxBodies() {
    return maxBodies;
  }

  /**
   * Broadphase body-pair capacity.
   *
   * @return pair capacity
   */
  public int maxBodyPairs() {
    return maxBodyPairs;
  }

  /**
   * Contact constraint capacity.
   *
   * @return contact capacity
   */
  public int maxContactConstraints() {
    return maxContactConstraints;
  }

  /**
   * Worker threads: 0 single-threaded, N &gt; 0 for N workers, -1 for auto.
   *
   * @return worker thread setting
   */
  public int workerThreads() {
    return workerThreads;
  }

  /**
   * Per-step native scratch memory in bytes.
   *
   * @return scratch bytes
   */
  public int tempAllocatorBytes() {
    return tempAllocatorBytes;
  }

  /**
   * Gravity magnitude in m/s², applied along -Z.
   *
   * @return gravity
   */
  public double gravity() {
    return gravity;
  }

  /**
   * Physics sub-steps per {@link SimWorld#step(double)} call.
   *
   * @return sub-steps
   */
  public int substeps() {
    return substeps;
  }

  /**
   * Game piece capacity.
   *
   * @return piece capacity
   */
  public int maxPieces() {
    return maxPieces;
  }

  /**
   * Impacts slower than this (m/s) do not bounce.
   *
   * @return threshold in m/s
   */
  public double minVelocityForRestitution() {
    return minVelocityForRestitution;
  }

  /**
   * Seconds a body must be at rest before sleeping.
   *
   * @return seconds
   */
  public double timeBeforeSleep() {
    return timeBeforeSleep;
  }

  /**
   * Velocity (m/s) below which a body counts as at rest.
   *
   * @return threshold in m/s
   */
  public double sleepVelocityThreshold() {
    return sleepVelocityThreshold;
  }

  /**
   * Solver velocity iterations.
   *
   * @return iterations
   */
  public int solverVelocitySteps() {
    return solverVelocitySteps;
  }

  /**
   * Solver position iterations.
   *
   * @return iterations
   */
  public int solverPositionSteps() {
    return solverPositionSteps;
  }

  @Override
  public String toString() {
    return "WorldConfig{maxBodies="
        + maxBodies
        + ", maxPieces="
        + maxPieces
        + ", workerThreads="
        + workerThreads
        + ", substeps="
        + substeps
        + ", gravity="
        + gravity
        + "}";
  }

  private static void requirePositive(String name, int value) {
    if (value <= 0) {
      throw new IllegalArgumentException(name + " must be > 0");
    }
  }

  private static void requireNonNegative(String name, double value) {
    if (!Double.isFinite(value) || value < 0.0) {
      throw new IllegalArgumentException(name + " must be finite and >= 0");
    }
  }

  /** Mutable builder for {@link WorldConfig}. Values are validated in {@link #build()}. */
  public static final class Builder {
    private int maxBodies = 4096;
    private int maxBodyPairs = 32768;
    private int maxContactConstraints = 16384;
    private int workerThreads = 2;
    private int tempAllocatorBytes = 32 * 1024 * 1024;
    private double gravity = STANDARD_GRAVITY;
    private int substeps = 5;
    private int maxPieces = 1024;
    private double minVelocityForRestitution = 0.2;
    private double timeBeforeSleep = 0.5;
    private double sleepVelocityThreshold = 0.03;
    private int solverVelocitySteps = 10;
    private int solverPositionSteps = 2;

    private Builder() {}

    /**
     * Sets the body capacity.
     *
     * @param value body capacity
     * @return this builder
     */
    public Builder maxBodies(int value) {
      maxBodies = value;
      return this;
    }

    /**
     * Sets the broadphase pair capacity.
     *
     * @param value pair capacity
     * @return this builder
     */
    public Builder maxBodyPairs(int value) {
      maxBodyPairs = value;
      return this;
    }

    /**
     * Sets the contact constraint capacity.
     *
     * @param value contact capacity
     * @return this builder
     */
    public Builder maxContactConstraints(int value) {
      maxContactConstraints = value;
      return this;
    }

    /**
     * Sets worker threads: 0 single-threaded, N &gt; 0 for N workers, -1 for auto.
     *
     * @param value worker threads
     * @return this builder
     */
    public Builder workerThreads(int value) {
      workerThreads = value;
      return this;
    }

    /**
     * Sets per-step scratch memory.
     *
     * @param value bytes
     * @return this builder
     */
    public Builder tempAllocatorBytes(int value) {
      tempAllocatorBytes = value;
      return this;
    }

    /**
     * Sets gravity magnitude (m/s², along -Z).
     *
     * @param value gravity
     * @return this builder
     */
    public Builder gravity(double value) {
      gravity = value;
      return this;
    }

    /**
     * Sets default sub-steps per step call.
     *
     * @param value sub-steps
     * @return this builder
     */
    public Builder substeps(int value) {
      substeps = value;
      return this;
    }

    /**
     * Sets game piece capacity.
     *
     * @param value piece capacity
     * @return this builder
     */
    public Builder maxPieces(int value) {
      maxPieces = value;
      return this;
    }

    /**
     * Sets the minimum impact speed (m/s) that bounces.
     *
     * @param value threshold in m/s
     * @return this builder
     */
    public Builder minVelocityForRestitution(double value) {
      minVelocityForRestitution = value;
      return this;
    }

    /**
     * Sets seconds at rest before sleeping.
     *
     * @param value seconds
     * @return this builder
     */
    public Builder timeBeforeSleep(double value) {
      timeBeforeSleep = value;
      return this;
    }

    /**
     * Sets the at-rest velocity threshold (m/s).
     *
     * @param value threshold
     * @return this builder
     */
    public Builder sleepVelocityThreshold(double value) {
      sleepVelocityThreshold = value;
      return this;
    }

    /**
     * Sets solver velocity iterations.
     *
     * @param value iterations
     * @return this builder
     */
    public Builder solverVelocitySteps(int value) {
      solverVelocitySteps = value;
      return this;
    }

    /**
     * Sets solver position iterations.
     *
     * @param value iterations
     * @return this builder
     */
    public Builder solverPositionSteps(int value) {
      solverPositionSteps = value;
      return this;
    }

    /**
     * Validates and builds the configuration.
     *
     * @return configuration
     * @throws IllegalArgumentException if any value is out of range
     */
    public WorldConfig build() {
      return new WorldConfig(this);
    }
  }
}
