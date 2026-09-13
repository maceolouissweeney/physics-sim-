package org.frcsim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;
import org.frcsim.jni.FrcSimJNI;

/**
 * Static field geometry of a {@link SimWorld}: ground, walls, and field elements. Coordinates use
 * the WPILib field frame (meters, Z up).
 */
public final class Field {
  private final SimWorld world;

  Field(SimWorld world) {
    this.world = world;
  }

  /**
   * Loads a {@code frcsim.field/1} JSON document (see docs/reference/field-json.md). May define
   * materials, statics, bounds, piece types, and piece spawns.
   *
   * @param json document text
   * @throws IllegalArgumentException if the document is malformed (the message names the location)
   */
  public void loadJson(String json) {
    Objects.requireNonNull(json, "json");
    FrcSimJNI.fieldLoadJson(world.nativeHandle(), json.getBytes(StandardCharsets.UTF_8));
  }

  /**
   * Loads a field JSON file.
   *
   * @param path file path
   * @throws IOException if the file cannot be read
   */
  public void loadJson(Path path) throws IOException {
    loadJson(Files.readString(path, StandardCharsets.UTF_8));
  }

  /**
   * Adds a large ground slab whose top surface is at {@code height}.
   *
   * @param height top surface height in meters
   * @param material surface material
   * @return field primitive index
   */
  public int addGround(double height, Material material) {
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.fieldAddGround(world.nativeHandle(), (float) height, material.id());
  }

  /**
   * Adds a static box rotated about the Z axis.
   *
   * @param name name for debugging
   * @param x center x
   * @param y center y
   * @param z center z
   * @param halfX half extent along x
   * @param halfY half extent along y
   * @param halfZ half extent along z
   * @param yawRadians rotation about +Z
   * @param material surface material
   * @return field primitive index
   */
  public int addBox(
      String name,
      double x,
      double y,
      double z,
      double halfX,
      double halfY,
      double halfZ,
      double yawRadians,
      Material material) {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.fieldAddBox(
        world.nativeHandle(),
        name,
        (float) x,
        (float) y,
        (float) z,
        (float) halfX,
        (float) halfY,
        (float) halfZ,
        0.0f,
        0.0f,
        (float) Math.sin(yawRadians / 2.0),
        (float) Math.cos(yawRadians / 2.0),
        material.id());
  }

  /**
   * Sets the playable bounds. Pieces that leave them become {@link PieceState#OUT_OF_BOUNDS}.
   *
   * @param minX minimum x
   * @param minY minimum y
   * @param minZ minimum z
   * @param maxX maximum x
   * @param maxY maximum y
   * @param maxZ maximum z
   */
  public void setBounds(
      double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
    FrcSimJNI.fieldSetBounds(
        world.nativeHandle(),
        (float) minX,
        (float) minY,
        (float) minZ,
        (float) maxX,
        (float) maxY,
        (float) maxZ);
  }
}
