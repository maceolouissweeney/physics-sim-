package org.frcsim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import org.frcsim.jni.FrcSimJNI;

/** Robots of a {@link SimWorld}. */
public final class Robots {
  private final SimWorld world;
  private final List<SwerveRobot> swerve = new ArrayList<>();

  Robots(SimWorld world) {
    this.world = world;
  }

  /**
   * Adds a swerve robot at rest on the carpet.
   *
   * @param config robot configuration (copied; later changes have no effect)
   * @param xMeters field x
   * @param yMeters field y
   * @param yawRadians heading
   * @return robot handle
   * @throws IllegalArgumentException if the configuration is invalid (the message names the value)
   */
  public SwerveRobot addSwerve(
      SwerveDriveConfig config, double xMeters, double yMeters, double yawRadians) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(config.bumperMaterial, "bumperMaterial");
    Material bumper = world.materials().get(config.bumperMaterial);
    long handle = world.nativeHandle();
    int index =
        FrcSimJNI.robotAddSwerve(
            handle,
            config.packRobot(),
            bumper.id(),
            config.packModules(),
            (float) xMeters,
            (float) yMeters,
            (float) yawRadians);
    SwerveRobot robot =
        new SwerveRobot(world, index, FrcSimJNI.robotIoBuffer(handle, index), config);
    swerve.add(robot);
    return robot;
  }

  /**
   * Returns a swerve robot by index.
   *
   * @param index robot index
   * @return robot handle
   */
  public SwerveRobot swerve(int index) {
    world.checkOpen();
    return swerve.get(index);
  }

  /**
   * All swerve robots in creation order.
   *
   * @return unmodifiable list
   */
  public List<SwerveRobot> swerveRobots() {
    return Collections.unmodifiableList(swerve);
  }
}
