package org.frcsim;

import java.util.Objects;
import org.frcsim.jni.FrcSimJNI;

/**
 * Scripted, infinitely massive bodies that push other bodies: test obstacles, benchmark plows,
 * moving field elements.
 */
public final class KinematicBodies {
  private final SimWorld world;

  KinematicBodies(SimWorld world) {
    this.world = world;
  }

  /**
   * Adds an axis-aligned kinematic box.
   *
   * @param xMeters center x
   * @param yMeters center y
   * @param zMeters center z
   * @param halfXMeters half extent x
   * @param halfYMeters half extent y
   * @param halfZMeters half extent z
   * @param material surface material
   * @return kinematic body index
   */
  public int addBox(
      double xMeters,
      double yMeters,
      double zMeters,
      double halfXMeters,
      double halfYMeters,
      double halfZMeters,
      Material material) {
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.kinematicAddBox(
        world.nativeHandle(),
        (float) xMeters,
        (float) yMeters,
        (float) zMeters,
        (float) halfXMeters,
        (float) halfYMeters,
        (float) halfZMeters,
        0f,
        0f,
        0f,
        1f,
        material.id());
  }

  /**
   * Moves a body so it reaches the pose at the end of the next step of {@code dtSeconds}. The
   * resulting velocity persists until the next call.
   *
   * @param index kinematic body index
   * @param xMeters target x
   * @param yMeters target y
   * @param zMeters target z
   * @param yawRadians target rotation about +Z
   * @param dtSeconds duration of the next step
   */
  public void moveTo(
      int index,
      double xMeters,
      double yMeters,
      double zMeters,
      double yawRadians,
      double dtSeconds) {
    FrcSimJNI.kinematicMoveTo(
        world.nativeHandle(),
        index,
        (float) xMeters,
        (float) yMeters,
        (float) zMeters,
        0f,
        0f,
        (float) Math.sin(yawRadians / 2.0),
        (float) Math.cos(yawRadians / 2.0),
        dtSeconds);
  }
}
