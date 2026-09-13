package org.frcsim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Zero-copy view of the native {@code frcsim_world_stats} block. Values refresh after every {@link
 * SimWorld#step} and after piece spawn/state changes. Accessors do not allocate.
 */
public final class WorldStats {
  // Mirrors frcsim_world_stats in frcsim_c.h; verified against native offsetof() by tests.
  static final int SIZE = 40;
  static final int OFFSET_TIME_SECONDS = 0;
  static final int OFFSET_LAST_STEP_WALL_SECONDS = 8;
  static final int OFFSET_SUBSTEP_COUNT = 16;
  static final int OFFSET_ACTIVE_BODIES = 24;
  static final int OFFSET_UPDATE_ERROR_FLAGS = 28;
  static final int OFFSET_PIECE_HIGH_WATER = 32;
  static final int OFFSET_PIECES_SIMULATED = 36;

  private final SimWorld world;
  private final ByteBuffer buffer;

  WorldStats(SimWorld world, ByteBuffer buffer) {
    if (buffer.capacity() != SIZE) {
      throw new FrcSimException(
          "native stats block is " + buffer.capacity() + " bytes, expected " + SIZE);
    }
    this.world = world;
    this.buffer = buffer.order(ByteOrder.nativeOrder());
  }

  /**
   * Simulated time since creation.
   *
   * @return seconds
   */
  public double timeSeconds() {
    world.checkOpen();
    return buffer.getDouble(OFFSET_TIME_SECONDS);
  }

  /**
   * Wall-clock duration of the most recent step.
   *
   * @return seconds
   */
  public double lastStepWallSeconds() {
    world.checkOpen();
    return buffer.getDouble(OFFSET_LAST_STEP_WALL_SECONDS);
  }

  /**
   * Physics sub-steps executed since creation.
   *
   * @return sub-step count
   */
  public long substepCount() {
    world.checkOpen();
    return buffer.getLong(OFFSET_SUBSTEP_COUNT);
  }

  /**
   * Awake rigid bodies after the most recent step. Sleeping bodies cost almost nothing.
   *
   * @return active body count
   */
  public int activeBodies() {
    world.checkOpen();
    return buffer.getInt(OFFSET_ACTIVE_BODIES);
  }

  /**
   * Union of native physics update error flags (non-zero means a capacity such as {@link
   * WorldConfig#maxContactConstraints()} overflowed and contacts were dropped).
   *
   * @return error flags
   */
  public int updateErrorFlags() {
    world.checkOpen();
    return buffer.getInt(OFFSET_UPDATE_ERROR_FLAGS);
  }

  /**
   * Number of piece indices ever used; piece data is valid for indices below this.
   *
   * @return piece high-water mark
   */
  public int pieceHighWater() {
    world.checkOpen();
    return buffer.getInt(OFFSET_PIECE_HIGH_WATER);
  }

  /**
   * Pieces currently simulated as rigid bodies (on field or airborne).
   *
   * @return simulated piece count
   */
  public int piecesSimulated() {
    world.checkOpen();
    return buffer.getInt(OFFSET_PIECES_SIMULATED);
  }
}
