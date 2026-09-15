package org.frcsim;

import java.util.Objects;

/**
 * Description of a game piece type, registered with {@link GamePieces#registerType}. Use the
 * factory methods; dimensions not used by the shape are ignored.
 *
 * @param name unique type name
 * @param shape collision shape
 * @param radiusMeters sphere or cylinder radius
 * @param halfHeightMeters cylinder half height
 * @param halfExtentXMeters box half extent x
 * @param halfExtentYMeters box half extent y
 * @param halfExtentZMeters box half extent z
 * @param massKg mass
 * @param material surface material
 * @param maxAngularVelocityRadPerSec angular velocity limit
 */
public record GamePieceTypeSpec(
    String name,
    GamePieceShape shape,
    double radiusMeters,
    double halfHeightMeters,
    double halfExtentXMeters,
    double halfExtentYMeters,
    double halfExtentZMeters,
    double massKg,
    Material material,
    double maxAngularVelocityRadPerSec) {

  /** Default angular velocity limit, high enough for shooter backspin. */
  public static final double DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 500.0;

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
   * @param radiusMeters radius
   * @param massKg mass
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec sphere(
      String name, double radiusMeters, double massKg, Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.SPHERE,
        radiusMeters,
        0,
        0,
        0,
        0,
        massKg,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC);
  }

  /**
   * A cylindrical piece with its axis along Z.
   *
   * @param name unique type name
   * @param radiusMeters radius
   * @param halfHeightMeters half height
   * @param massKg mass
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec cylinder(
      String name, double radiusMeters, double halfHeightMeters, double massKg, Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.CYLINDER,
        radiusMeters,
        halfHeightMeters,
        0,
        0,
        0,
        massKg,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC);
  }

  /**
   * A box-shaped piece.
   *
   * @param name unique type name
   * @param halfXMeters half extent x
   * @param halfYMeters half extent y
   * @param halfZMeters half extent z
   * @param massKg mass
   * @param material surface material
   * @return spec
   */
  public static GamePieceTypeSpec box(
      String name,
      double halfXMeters,
      double halfYMeters,
      double halfZMeters,
      double massKg,
      Material material) {
    return new GamePieceTypeSpec(
        name,
        GamePieceShape.BOX,
        0,
        0,
        halfXMeters,
        halfYMeters,
        halfZMeters,
        massKg,
        material,
        DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC);
  }

  /**
   * Returns a copy with a different angular velocity limit.
   *
   * @param value limit in rad/s
   * @return modified copy
   */
  public GamePieceTypeSpec withMaxAngularVelocityRadPerSec(double value) {
    return new GamePieceTypeSpec(
        name,
        shape,
        radiusMeters,
        halfHeightMeters,
        halfExtentXMeters,
        halfExtentYMeters,
        halfExtentZMeters,
        massKg,
        material,
        value);
  }
}
