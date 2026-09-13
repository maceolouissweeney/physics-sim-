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
   * @param x center x
   * @param y center y
   * @param z center z
   * @param halfX half extent x
   * @param halfY half extent y
   * @param halfZ half extent z
   * @param material surface material
   * @return kinematic body index
   */
  public int addBox(
      double x, double y, double z, double halfX, double halfY, double halfZ, Material material) {
    Objects.requireNonNull(material, "material");
    return FrcSimJNI.kinematicAddBox(
        world.nativeHandle(),
        (float) x,
        (float) y,
        (float) z,
        (float) halfX,
        (float) halfY,
        (float) halfZ,
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
   * @param x target x
   * @param y target y
   * @param z target z
   * @param yawRadians target rotation about +Z
   * @param dtSeconds duration of the next step
   */
  public void moveTo(int index, double x, double y, double z, double yawRadians, double dtSeconds) {
    FrcSimJNI.kinematicMoveTo(
        world.nativeHandle(),
        index,
        (float) x,
        (float) y,
        (float) z,
        0f,
        0f,
        (float) Math.sin(yawRadians / 2.0),
        (float) Math.cos(yawRadians / 2.0),
        dtSeconds);
  }
}
