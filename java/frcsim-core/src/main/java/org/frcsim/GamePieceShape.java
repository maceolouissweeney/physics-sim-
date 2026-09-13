package org.frcsim;

/** Collision shape of a game piece type. Codes match {@code frcsim_piece_shape} in the C ABI. */
public enum GamePieceShape {
  /** Sphere (e.g. REBUILT fuel). */
  SPHERE,
  /** Cylinder with its axis along Z. */
  CYLINDER,
  /** Box. */
  BOX
}
