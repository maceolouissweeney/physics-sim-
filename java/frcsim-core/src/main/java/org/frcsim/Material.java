package org.frcsim;

/**
 * Handle to a material registered in a {@link SimWorld}.
 *
 * @param id native material id (0..63)
 * @param name material name
 */
public record Material(int id, String name) {}
