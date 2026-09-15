package org.frcsim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;
import org.frcsim.jni.FrcSimJNI;

/**
 * Static field geometry of a {@link SimWorld}: ground, walls, and field elements. Coordinates use
 * the WPILib field frame (Z up).
 */
public final class Field {
  private final SimWorld world;

  Field(SimWorld world) {
    this.world = world;
  }

  /**
   * Loads a {@code frcsim.field} JSON document (see docs/reference/field-json.md). May define
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
   * Adds a large ground slab whose top surface is at {@code heightMeters}.
   *
   * @param heightMeters top surface height
   * @param material surface material
   * @return field primitive index
   */
  public int addGround(double heightMeters, Material material) {
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.fieldAddGround(world.nativeHandle(), (float) heightMeters, material.id());
  }

  /**
   * Adds a static box rotated about the Z axis.
   *
   * @param name name for debugging
   * @param xMeters center x
   * @param yMeters center y
   * @param zMeters center z
   * @param halfXMeters half extent along x
   * @param halfYMeters half extent along y
   * @param halfZMeters half extent along z
   * @param yawRadians rotation about +Z
   * @param material surface material
   * @return field primitive index
   */
  public int addBox(
      String name,
      double xMeters,
      double yMeters,
      double zMeters,
      double halfXMeters,
      double halfYMeters,
      double halfZMeters,
      double yawRadians,
      Material material) {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.fieldAddBox(
        world.nativeHandle(),
        name,
        (float) xMeters,
        (float) yMeters,
        (float) zMeters,
        (float) halfXMeters,
        (float) halfYMeters,
        (float) halfZMeters,
        0.0f,
        0.0f,
        (float) Math.sin(yawRadians / 2.0),
        (float) Math.cos(yawRadians / 2.0),
        material.id());
  }

  /**
   * Sets the playable bounds. Pieces that leave them become {@link PieceState#OUT_OF_BOUNDS}.
   *
   * @param minXMeters minimum x
   * @param minYMeters minimum y
   * @param minZMeters minimum z
   * @param maxXMeters maximum x
   * @param maxYMeters maximum y
   * @param maxZMeters maximum z
   */
  public void setBoundsMeters(
      double minXMeters,
      double minYMeters,
      double minZMeters,
      double maxXMeters,
      double maxYMeters,
      double maxZMeters) {
    FrcSimJNI.fieldSetBounds(
        world.nativeHandle(),
        (float) minXMeters,
        (float) minYMeters,
        (float) minZMeters,
        (float) maxXMeters,
        (float) maxYMeters,
        (float) maxZMeters);
  }
}
