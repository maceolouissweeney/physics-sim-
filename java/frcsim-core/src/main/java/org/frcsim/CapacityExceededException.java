package org.frcsim;

/**
 * A capacity fixed at world creation (pieces, bodies, materials, piece types) is exhausted. Raise
 * the matching {@link WorldConfig} limit.
 */
public class CapacityExceededException extends FrcSimException {
  private static final long serialVersionUID = 1L;

  /**
   * Creates an exception.
   *
   * @param message description of the exhausted capacity
   */
  public CapacityExceededException(String message) {
    super(message);
  }
}
