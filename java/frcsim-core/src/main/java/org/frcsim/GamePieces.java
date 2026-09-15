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
  private final FloatBuffer positionsMeters;
  private final ByteBuffer states;
  private final int capacity;
  private final float[] scratchPositionMeters = new float[3];
  private final int[] scratchIndex = new int[1];

  GamePieces(SimWorld world, ByteBuffer positionsMeters, ByteBuffer states, int capacity) {
    if (positionsMeters.capacity() != capacity * 3 * Float.BYTES || states.capacity() != capacity) {
      throw new FrcSimException("native piece buffers do not match maxPieces = " + capacity);
    }
    this.world = world;
    this.positionsMeters = positionsMeters.order(ByteOrder.nativeOrder()).asFloatBuffer();
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
            (float) spec.radiusMeters(),
            (float) spec.halfHeightMeters(),
            (float) spec.halfExtentXMeters(),
            (float) spec.halfExtentYMeters(),
            (float) spec.halfExtentZMeters(),
            (float) spec.massKg(),
            spec.material().id(),
            (float) spec.maxAngularVelocityRadPerSec());
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
   * @param xMeters position x
   * @param yMeters position y
   * @param zMeters position z
   * @return piece index
   * @throws CapacityExceededException if {@link WorldConfig#maxPieces()} is reached
   */
  public int spawn(GamePieceType type, double xMeters, double yMeters, double zMeters) {
    Objects.requireNonNull(type, "type");
    scratchPositionMeters[0] = (float) xMeters;
    scratchPositionMeters[1] = (float) yMeters;
    scratchPositionMeters[2] = (float) zMeters;
    FrcSimJNI.piecesSpawn(
        world.nativeHandle(), type.id(), scratchPositionMeters, null, scratchIndex);
    return scratchIndex[0];
  }

  /**
   * Spawns many pieces at rest.
   *
   * @param type piece type
   * @param positionsXyzMeters xyz triples
   * @return new piece indices
   * @throws CapacityExceededException if capacity would be exceeded (nothing is spawned)
   */
  public int[] spawn(GamePieceType type, float[] positionsXyzMeters) {
    Objects.requireNonNull(positionsXyzMeters, "positionsXyzMeters");
    int[] indices = new int[positionsXyzMeters.length / 3];
    spawn(type, positionsXyzMeters, null, indices);
    return indices;
  }

  /**
   * Spawns many pieces with optional initial velocities.
   *
   * @param type piece type
   * @param positionsXyzMeters xyz triples
   * @param velocitiesXyzMetersPerSec xyz triples matching positions, or null for at rest
   * @param outIndices receives the new indices, or null
   * @throws CapacityExceededException if capacity would be exceeded (nothing is spawned)
   */
  public void spawn(
      GamePieceType type,
      float[] positionsXyzMeters,
      float[] velocitiesXyzMetersPerSec,
      int[] outIndices) {
    Objects.requireNonNull(type, "type");
    Objects.requireNonNull(positionsXyzMeters, "positionsXyzMeters");
    FrcSimJNI.piecesSpawn(
        world.nativeHandle(), type.id(), positionsXyzMeters, velocitiesXyzMetersPerSec, outIndices);
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
   * @param xMeters position x
   * @param yMeters position y
   * @param zMeters position z
   * @param vxMetersPerSec velocity x
   * @param vyMetersPerSec velocity y
   * @param vzMetersPerSec velocity z
   */
  public void teleport(
      int index,
      double xMeters,
      double yMeters,
      double zMeters,
      double vxMetersPerSec,
      double vyMetersPerSec,
      double vzMetersPerSec) {
    FrcSimJNI.pieceTeleport(
        world.nativeHandle(),
        index,
        (float) xMeters,
        (float) yMeters,
        (float) zMeters,
        (float) vxMetersPerSec,
        (float) vyMetersPerSec,
        (float) vzMetersPerSec,
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
   * @return meters
   */
  public double xMeters(int index) {
    return positionsMeters.get(3 * checkIndex(index));
  }

  /**
   * Piece y position after the latest step.
   *
   * @param index piece index
   * @return meters
   */
  public double yMeters(int index) {
    return positionsMeters.get(3 * checkIndex(index) + 1);
  }

  /**
   * Piece z position after the latest step.
   *
   * @param index piece index
   * @return meters
   */
  public double zMeters(int index) {
    return positionsMeters.get(3 * checkIndex(index) + 2);
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
   * @param destinationMeters array with room for {@code 3 * count()} floats
   * @return number of pieces copied
   */
  public int copyPositionsMeters(float[] destinationMeters) {
    world.checkOpen();
    int n = Math.min(count(), destinationMeters.length / 3);
    positionsMeters.get(0, destinationMeters, 0, 3 * n);
    return n;
  }

  /**
   * Copies positions of pieces in {@code state} as packed xyz triples, in index order. Does not
   * allocate. Typical use: telemetry of every {@link PieceState#ON_FIELD} piece.
   *
   * @param state state to include
   * @param destinationMeters array with room for {@code 3 * count()} floats
   * @return number of pieces copied
   */
  public int copyPositionsMeters(PieceState state, float[] destinationMeters) {
    world.checkOpen();
    int n = count();
    int limit = destinationMeters.length / 3;
    byte code = (byte) state.code();
    int copied = 0;
    for (int i = 0; i < n && copied < limit; i++) {
      if (states.get(i) == code) {
        int src = 3 * i;
        int dst = 3 * copied;
        destinationMeters[dst] = positionsMeters.get(src);
        destinationMeters[dst + 1] = positionsMeters.get(src + 1);
        destinationMeters[dst + 2] = positionsMeters.get(src + 2);
        copied++;
      }
    }
    return copied;
  }

  private int checkIndex(int index) {
    world.checkOpen();
    if (index < 0 || index >= capacity) {
      throw new IndexOutOfBoundsException("piece index " + index + " out of range 0.." + capacity);
    }
    return index;
  }
}
