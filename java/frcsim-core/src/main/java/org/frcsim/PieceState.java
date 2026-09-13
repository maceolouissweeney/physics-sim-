package org.frcsim;

/** Lifecycle state of a game piece. Codes match {@code frcsim_piece_state} in the C ABI. */
public enum PieceState {
  /** Not spawned, or despawned; the index may be reused. */
  INACTIVE,
  /** Simulated rigid body on the field. */
  ON_FIELD,
  /** Simulated rigid body in flight. */
  AIRBORNE,
  /** Held by a robot; not simulated. */
  IN_ROBOT,
  /** Counted by a goal; not simulated. */
  SCORED,
  /** Left the field bounds; not simulated. */
  OUT_OF_BOUNDS;

  private static final PieceState[] VALUES = values();

  /**
   * Native code of this state.
   *
   * @return state code
   */
  public int code() {
    return ordinal();
  }

  /**
   * Converts a native state code. Does not allocate.
   *
   * @param code state code
   * @return state
   * @throws IllegalArgumentException for unknown codes
   */
  public static PieceState fromCode(int code) {
    if (code < 0 || code >= VALUES.length) {
      throw new IllegalArgumentException("unknown piece state code " + code);
    }
    return VALUES[code];
  }

  /**
   * Whether pieces in this state are simulated rigid bodies.
   *
   * @return true for {@link #ON_FIELD} and {@link #AIRBORNE}
   */
  public boolean isSimulated() {
    return this == ON_FIELD || this == AIRBORNE;
  }
}
