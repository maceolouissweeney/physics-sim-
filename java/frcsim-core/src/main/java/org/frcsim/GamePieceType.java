package org.frcsim;

/**
 * Handle to a game piece type registered in a {@link SimWorld}.
 *
 * @param id native type id
 * @param name type name
 */
public record GamePieceType(int id, String name) {}
