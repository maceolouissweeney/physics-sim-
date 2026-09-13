package org.frcsim;

import java.util.Objects;

/**
 * Description of a game piece type, registered with {@link GamePieces#registerType}. Use the
 * factory methods; dimensions not used by the shape are ignored.
 *
 * @param name unique type name
 * @param shape collision shape
 * @param radius sphere or cylinder radius (m)
 * @param halfHeight cylinder half height (m)
 * @param halfExtentX box half extent x (m)
 * @param halfExtentY box half extent y (m)
 * @param halfExtentZ box half extent z (m)
 * @param mass mass (kg)
 * @param material surface material
 * @param maxAngularVelocity angular velocity limit (rad/s)
 */
public record GamePieceTypeSpec(
    String name,
    GamePieceShape shape,
    double radius,
    double halfHeight,
    double halfExtentX,
    double halfExtentY,
    double halfExtentZ,
    double mass,
    Material material,
    double maxAngularVelocity) {

  /** Default angular velocity limit, high enough for shooter backspin. */
  public static final double DEFAULT_MAX_ANGULAR_VELOCITY = 500.0;

  /** Validates non-null fields. Numeric ranges are validated natively on registration. */
  public GamePieceTypeSpec {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(shape, "shape");
    Objects.requireNonNull(material, "material");
  }

  /**
   * A spherical piece.
   *
   * @param name unique type name
   * @param radius radius (m)
   * @param mass mass (kg)
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec sphere(
      String name, double radius, double mass, Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.SPHERE,
        radius,
        0,
        0,
        0,
        0,
        mass,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY);
  }

  /**
   * A cylindrical piece with its axis along Z.
   *
   * @param name unique type name
   * @param radius radius (m)
   * @param halfHeight half height (m)
   * @param mass mass (kg)
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec cylinder(
      String name, double radius, double halfHeight, double mass, Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.CYLINDER,
        radius,
        halfHeight,
        0,
        0,
        0,
        mass,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY);
  }

  /**
   * A box-shaped piece.
   *
   * @param name unique type name
   * @param halfX half extent x (m)
   * @param halfY half extent y (m)
   * @param halfZ half extent z (m)
   * @param mass mass (kg)
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec box(
      String name, double halfX, double halfY, double halfZ, double mass, Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.BOX,
        0,
        0,
        halfX,
        halfY,
        halfZ,
        mass,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY);
  }

  /**
   * Returns a copy with a different angular velocity limit.
   *
   * @param value limit in rad/s
   * @return modified copy
   */
  public GamePieceTypeSpec withMaxAngularVelocity(double value) {
    return new GamePieceTypeSpec(
        name,
        shape,
        radius,
        halfHeight,
        halfExtentX,
        halfExtentY,
        halfExtentZ,
        mass,
        material,
        value);
  }
}
