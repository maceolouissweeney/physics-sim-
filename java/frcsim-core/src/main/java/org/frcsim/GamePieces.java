package org.frcsim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Objects;
import java.util.Optional;
import org.frcsim.jni.FrcSimJNI;

/**
 * Game pieces of a {@link SimWorld}.
 *
 * <p>Pieces are identified by a stable integer index. Positions and states are zero-copy views of
 * native memory refreshed after every step; reading them does not allocate or cross into native
 * code. Valid indices are {@code [0, count())}.
 *
 * <p>Not thread-safe (the single-piece {@link #spawn(GamePieceType, double, double, double)} reuses
 * scratch arrays to avoid allocation).
 */
public final class GamePieces {
  private final SimWorld world;
  private final FloatBuffer positions;
  private final ByteBuffer states;
  private final int capacity;
  private final float[] scratchPosition = new float[3];
  private final int[] scratchIndex = new int[1];

  GamePieces(SimWorld world, ByteBuffer positions, ByteBuffer states, int capacity) {
    if (positions.capacity() != capacity * 3 * Float.BYTES || states.capacity() != capacity) {
      throw new FrcSimException("native piece buffers do not match maxPieces = " + capacity);
    }
    this.world = world;
    this.positions = positions.order(ByteOrder.nativeOrder()).asFloatBuffer();
    this.states = states;
    this.capacity = capacity;
  }

  /**
   * Registers a piece type.
   *
   * @param spec type description
   * @return type handle
   * @throws IllegalArgumentException for invalid dimensions or a duplicate name
   */
  public GamePieceType registerType(GamePieceTypeSpec spec) {
    Objects.requireNonNull(spec, "spec");
    int id =
        FrcSimJNI.pieceTypeAdd(
            world.nativeHandle(),
            spec.name(),
            spec.shape().ordinal(),
            (float) spec.radius(),
            (float) spec.halfHeight(),
            (float) spec.halfExtentX(),
            (float) spec.halfExtentY(),
            (float) spec.halfExtentZ(),
            (float) spec.mass(),
            spec.material().id(),
            (float) spec.maxAngularVelocity());
    return new GamePieceType(id, spec.name());
  }

  /**
   * Looks up a piece type by name (for example one defined in a field JSON file).
   *
   * @param name type name
   * @return type handle, or empty
   */
  public Optional<GamePieceType> findType(String name) {
    Objects.requireNonNull(name, "name");
    int id = FrcSimJNI.pieceTypeFind(world.nativeHandle(), name);
    return id < 0 ? Optional.empty() : Optional.of(new GamePieceType(id, name));
  }

  /**
   * Spawns one piece at rest. Does not allocate.
   *
   * @param type piece type
   * @param x position x (m)
   * @param y position y (m)
   * @param z position z (m)
   * @return piece index
   * @throws CapacityExceededException if {@link WorldConfig#maxPieces()} is reached
   */
  public int spawn(GamePieceType type, double x, double y, double z) {
    Objects.requireNonNull(type, "type");
    scratchPosition[0] = (float) x;
    scratchPosition[1] = (float) y;
    scratchPosition[2] = (float) z;
    FrcSimJNI.piecesSpawn(world.nativeHandle(), type.id(), scratchPosition, null, scratchIndex);
    return scratchIndex[0];
  }

  /**
   * Spawns many pieces at rest.
   *
   * @param type piece type
   * @param positionsXyz xyz triples
   * @return new piece indices
   * @throws CapacityExceededException if capacity would be exceeded (nothing is spawned)
   */
  public int[] spawn(GamePieceType type, float[] positionsXyz) {
    Objects.requireNonNull(positionsXyz, "positionsXyz");
    int[] indices = new int[positionsXyz.length / 3];
    spawn(type, positionsXyz, null, indices);
    return indices;
  }

  /**
   * Spawns many pieces with optional initial velocities.
   *
   * @param type piece type
   * @param positionsXyz xyz triples
   * @param velocitiesXyz xyz triples matching positions, or null for at rest
   * @param outIndices receives the new indices, or null
   * @throws CapacityExceededException if capacity would be exceeded (nothing is spawned)
   */
  public void spawn(
      GamePieceType type, float[] positionsXyz, float[] velocitiesXyz, int[] outIndices) {
    Objects.requireNonNull(type, "type");
    Objects.requireNonNull(positionsXyz, "positionsXyz");
    FrcSimJNI.piecesSpawn(world.nativeHandle(), type.id(), positionsXyz, velocitiesXyz, outIndices);
  }

  /**
   * Removes a piece from the simulation; its index may be reused by a later spawn.
   *
   * @param index piece index
   * @throws java.util.NoSuchElementException if the piece is not spawned
   */
  public void despawn(int index) {
    FrcSimJNI.pieceDespawn(world.nativeHandle(), index);
  }

  /**
   * Changes a spawned piece's state (for example {@link PieceState#IN_ROBOT} when intaken).
   * Non-simulated states remove the piece from physics until it returns to a simulated state.
   *
   * @param index piece index
   * @param state new state; use {@link #despawn} for {@link PieceState#INACTIVE}
   */
  public void setState(int index, PieceState state) {
    Objects.requireNonNull(state, "state");
    FrcSimJNI.pieceSetState(world.nativeHandle(), index, state.code());
  }

  /**
   * Places a spawned piece on the field with a velocity, putting it {@link PieceState#ON_FIELD}.
   *
   * @param index piece index
   * @param x position x
   * @param y position y
   * @param z position z
   * @param vx velocity x (m/s)
   * @param vy velocity y (m/s)
   * @param vz velocity z (m/s)
   */
  public void teleport(int index, double x, double y, double z, double vx, double vy, double vz) {
    FrcSimJNI.pieceTeleport(
        world.nativeHandle(),
        index,
        (float) x,
        (float) y,
        (float) z,
        (float) vx,
        (float) vy,
        (float) vz,
        0f,
        0f,
        0f);
  }

  /**
   * Maximum number of pieces ({@link WorldConfig#maxPieces()}).
   *
   * @return capacity
   */
  public int capacity() {
    return capacity;
  }

  /**
   * Number of piece indices in use; iterate {@code [0, count())}.
   *
   * @return piece index high-water mark
   */
  public int count() {
    return world.stats().pieceHighWater();
  }

  /**
   * Piece x position after the latest step.
   *
   * @param index piece index
   * @return x in meters
   */
  public double x(int index) {
    return positions.get(3 * checkIndex(index));
  }

  /**
   * Piece y position after the latest step.
   *
   * @param index piece index
   * @return y in meters
   */
  public double y(int index) {
    return positions.get(3 * checkIndex(index) + 1);
  }

  /**
   * Piece z position after the latest step.
   *
   * @param index piece index
   * @return z in meters
   */
  public double z(int index) {
    return positions.get(3 * checkIndex(index) + 2);
  }

  /**
   * Piece state.
   *
   * @param index piece index
   * @return state
   */
  public PieceState state(int index) {
    return PieceState.fromCode(states.get(checkIndex(index)));
  }

  /**
   * Counts pieces in a state. Does not allocate.
   *
   * @param state state to count
   * @return number of pieces
   */
  public int countInState(PieceState state) {
    world.checkOpen();
    int n = count();
    int code = state.code();
    int total = 0;
    for (int i = 0; i < n; i++) {
      if (states.get(i) == code) {
        total++;
      }
    }
    return total;
  }

  /**
   * Copies positions of indices {@code [0, count())} as xyz triples. Does not allocate.
   *
   * @param destination array with room for {@code 3 * count()} floats
   * @return number of pieces copied
   */
  public int copyPositions(float[] destination) {
    world.checkOpen();
    int n = Math.min(count(), destination.length / 3);
    positions.get(0, destination, 0, 3 * n);
    return n;
  }

  private int checkIndex(int index) {
    world.checkOpen();
    if (index < 0 || index >= capacity) {
      throw new IndexOutOfBoundsException("piece index " + index + " out of range 0.." + capacity);
    }
    return index;
  }
}
