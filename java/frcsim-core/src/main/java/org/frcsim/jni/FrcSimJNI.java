package org.frcsim.jni;

import java.nio.ByteBuffer;

/**
 * Raw JNI declarations mirroring the C ABI in {@code native/include/frcsim/frcsim_c.h}. Internal:
 * callers must go through the public API, which ensures the library is loaded and handles are open.
 *
 * <p>Native failures surface as {@link IllegalArgumentException}, {@link IllegalStateException},
 * {@link java.util.NoSuchElementException}, {@link org.frcsim.CapacityExceededException}, {@link
 * OutOfMemoryError}, or {@link org.frcsim.FrcSimException}. Lookup functions ({@code *Find}) return
 * -1 when nothing matches.
 */
public final class FrcSimJNI {
  private FrcSimJNI() {}

  // ---- Library ------------------------------------------------------------------------------

  /**
   * Native library version string.
   *
   * @return version, e.g. {@code "0.1.0"}
   */
  public static native String versionString();

  /**
   * Native C ABI version.
   *
   * @return ABI version
   */
  public static native int abiVersion();

  // ---- World --------------------------------------------------------------------------------

  /**
   * Creates a world; see {@code frcsim_world_config} for parameter meanings.
   *
   * @param maxBodies body capacity
   * @param maxBodyPairs body pair capacity
   * @param maxContactConstraints contact constraint capacity
   * @param workerThreads worker threads (0 single-threaded, -1 auto)
   * @param tempAllocatorBytes per-step scratch memory
   * @param gravityZMetersPerSecSq gravity along +Z
   * @param maxPieces game piece capacity
   * @param minVelocityForRestitutionMetersPerSec impacts slower than this don't bounce
   * @param timeBeforeSleepSeconds time at rest before sleeping
   * @param sleepVelocityThresholdMetersPerSec velocity considered at rest
   * @param solverVelocitySteps solver velocity iterations
   * @param solverPositionSteps solver position iterations
   * @return opaque native handle, never 0
   */
  public static native long worldCreate(
      int maxBodies,
      int maxBodyPairs,
      int maxContactConstraints,
      int workerThreads,
      int tempAllocatorBytes,
      double gravityZMetersPerSecSq,
      int maxPieces,
      float minVelocityForRestitutionMetersPerSec,
      float timeBeforeSleepSeconds,
      float sleepVelocityThresholdMetersPerSec,
      int solverVelocitySteps,
      int solverPositionSteps);

  /**
   * Destroys a world. A 0 handle is ignored.
   *
   * @param world native handle
   */
  public static native void worldDestroy(long world);

  /**
   * Steps a world.
   *
   * @param world native handle
   * @param dtSeconds time to advance
   * @param substeps number of physics steps
   */
  public static native void worldStep(long world, double dtSeconds, int substeps);

  /**
   * Rebuilds broadphase trees.
   *
   * @param world native handle
   */
  public static native void worldOptimizeBroadPhase(long world);

  /**
   * Direct buffer over the native {@code frcsim_world_stats} block (native byte order).
   *
   * @param world native handle
   * @return buffer valid until the world is destroyed
   */
  public static native ByteBuffer worldStatsBuffer(long world);

  /**
   * Native stats layout: {@code [size, offsets of each field in declaration order]}.
   *
   * @return layout array
   */
  public static native int[] worldStatsLayout();

  // ---- Materials ----------------------------------------------------------------------------

  /**
   * Adds or updates a material.
   *
   * @param world native handle
   * @param name material name
   * @param friction friction coefficient
   * @param restitution restitution coefficient
   * @return material id
   */
  public static native int materialAdd(long world, String name, float friction, float restitution);

  /**
   * Finds a material.
   *
   * @param world native handle
   * @param name material name
   * @return material id, or -1 if not found
   */
  public static native int materialFind(long world, String name);

  /**
   * Overrides a material pair.
   *
   * @param world native handle
   * @param a material id
   * @param b material id
   * @param friction combined friction
   * @param restitution combined restitution
   */
  public static native void materialSetPair(
      long world, int a, int b, float friction, float restitution);

  // ---- Field --------------------------------------------------------------------------------

  /**
   * Loads a field JSON document.
   *
   * @param world native handle
   * @param jsonUtf8 UTF-8 bytes of the document
   */
  public static native void fieldLoadJson(long world, byte[] jsonUtf8);

  /**
   * Adds a ground slab.
   *
   * @param world native handle
   * @param heightMeters top surface height
   * @param material material id
   * @return field primitive index
   */
  public static native int fieldAddGround(long world, float heightMeters, int material);

  /**
   * Adds a static box.
   *
   * @param world native handle
   * @param name primitive name
   * @param centerXMeters center x
   * @param centerYMeters center y
   * @param centerZMeters center z
   * @param halfXMeters half extent x
   * @param halfYMeters half extent y
   * @param halfZMeters half extent z
   * @param qx rotation quaternion x
   * @param qy rotation quaternion y
   * @param qz rotation quaternion z
   * @param qw rotation quaternion w
   * @param material material id
   * @return field primitive index
   */
  public static native int fieldAddBox(
      long world,
      String name,
      float centerXMeters,
      float centerYMeters,
      float centerZMeters,
      float halfXMeters,
      float halfYMeters,
      float halfZMeters,
      float qx,
      float qy,
      float qz,
      float qw,
      int material);

  /**
   * Sets field bounds.
   *
   * @param world native handle
   * @param minXMeters min x
   * @param minYMeters min y
   * @param minZMeters min z
   * @param maxXMeters max x
   * @param maxYMeters max y
   * @param maxZMeters max z
   */
  public static native void fieldSetBounds(
      long world,
      float minXMeters,
      float minYMeters,
      float minZMeters,
      float maxXMeters,
      float maxYMeters,
      float maxZMeters);

  // ---- Game pieces --------------------------------------------------------------------------

  /**
   * Registers a piece type.
   *
   * @param world native handle
   * @param name unique name
   * @param shape shape code
   * @param radiusMeters radius
   * @param halfHeightMeters cylinder half height
   * @param halfXMeters box half extent x
   * @param halfYMeters box half extent y
   * @param halfZMeters box half extent z
   * @param massKg mass
   * @param material material id
   * @param maxAngularVelocityRadPerSec max angular velocity
   * @return piece type id
   */
  public static native int pieceTypeAdd(
      long world,
      String name,
      int shape,
      float radiusMeters,
      float halfHeightMeters,
      float halfXMeters,
      float halfYMeters,
      float halfZMeters,
      float massKg,
      int material,
      float maxAngularVelocityRadPerSec);

  /**
   * Finds a piece type.
   *
   * @param world native handle
   * @param name type name
   * @return type id, or -1 if not found
   */
  public static native int pieceTypeFind(long world, String name);

  /**
   * Spawns pieces.
   *
   * @param world native handle
   * @param type piece type id
   * @param positionsXyzMeters xyz triples
   * @param velocitiesXyzMetersPerSec xyz triples or null
   * @param outIndices receives indices, or null
   */
  public static native void piecesSpawn(
      long world,
      int type,
      float[] positionsXyzMeters,
      float[] velocitiesXyzMetersPerSec,
      int[] outIndices);

  /**
   * Despawns a piece.
   *
   * @param world native handle
   * @param index piece index
   */
  public static native void pieceDespawn(long world, int index);

  /**
   * Sets a piece state.
   *
   * @param world native handle
   * @param index piece index
   * @param state state code
   */
  public static native void pieceSetState(long world, int index, int state);

  /**
   * Teleports a piece.
   *
   * @param world native handle
   * @param index piece index
   * @param xMeters position x
   * @param yMeters position y
   * @param zMeters position z
   * @param vxMetersPerSec linear velocity x
   * @param vyMetersPerSec linear velocity y
   * @param vzMetersPerSec linear velocity z
   * @param wxRadPerSec angular velocity x
   * @param wyRadPerSec angular velocity y
   * @param wzRadPerSec angular velocity z
   */
  public static native void pieceTeleport(
      long world,
      int index,
      float xMeters,
      float yMeters,
      float zMeters,
      float vxMetersPerSec,
      float vyMetersPerSec,
      float vzMetersPerSec,
      float wxRadPerSec,
      float wyRadPerSec,
      float wzRadPerSec);

  /**
   * Direct buffer over piece positions in meters (3 floats per index, native byte order).
   *
   * @param world native handle
   * @return buffer valid until the world is destroyed
   */
  public static native ByteBuffer piecePositionsBuffer(long world);

  /**
   * Direct buffer over piece states (1 byte per index).
   *
   * @param world native handle
   * @return buffer valid until the world is destroyed
   */
  public static native ByteBuffer pieceStatesBuffer(long world);

  // ---- Robots -------------------------------------------------------------------------------

  /**
   * Adds a swerve robot.
   *
   * @param world native handle
   * @param robotParams packed robot fields ({@code SwerveDriveConfig.packRobot})
   * @param bumperMaterial bumper material id
   * @param moduleParams packed modules ({@code SwerveDriveConfig.packModules})
   * @param xMeters field x
   * @param yMeters field y
   * @param yawRadians heading
   * @return robot index
   */
  public static native int robotAddSwerve(
      long world,
      float[] robotParams,
      int bumperMaterial,
      float[] moduleParams,
      float xMeters,
      float yMeters,
      float yawRadians);

  /**
   * Direct buffer over a robot's {@code frcsim_swerve_robot_io} block (native byte order).
   *
   * @param world native handle
   * @param robot robot index
   * @return buffer valid until the world is destroyed
   */
  public static native ByteBuffer robotIoBuffer(long world, int robot);

  /**
   * Places a robot on the carpet at rest.
   *
   * @param world native handle
   * @param robot robot index
   * @param xMeters field x
   * @param yMeters field y
   * @param yawRadians heading
   */
  public static native void robotResetPose(
      long world, int robot, float xMeters, float yMeters, float yawRadians);

  /**
   * Native robot I/O layout: {@code [robot size, yaw, qx, vx, wx, battery volts, brownout, module
   * count, modules, module size, drive rotor position, steer angle, drive rotor velocity, steer
   * velocity, drive applied volts, normal force, slip speed, gyro yaw]}.
   *
   * @return layout array
   */
  public static native int[] robotIoLayout();

  // ---- Kinematic bodies ---------------------------------------------------------------------

  /**
   * Adds a kinematic box.
   *
   * @param world native handle
   * @param centerXMeters center x
   * @param centerYMeters center y
   * @param centerZMeters center z
   * @param halfXMeters half extent x
   * @param halfYMeters half extent y
   * @param halfZMeters half extent z
   * @param qx rotation quaternion x
   * @param qy rotation quaternion y
   * @param qz rotation quaternion z
   * @param qw rotation quaternion w
   * @param material material id
   * @return kinematic body index
   */
  public static native int kinematicAddBox(
      long world,
      float centerXMeters,
      float centerYMeters,
      float centerZMeters,
      float halfXMeters,
      float halfYMeters,
      float halfZMeters,
      float qx,
      float qy,
      float qz,
      float qw,
      int material);

  /**
   * Moves a kinematic body over the next step.
   *
   * @param world native handle
   * @param index kinematic body index
   * @param xMeters target x
   * @param yMeters target y
   * @param zMeters target z
   * @param qx target rotation x
   * @param qy target rotation y
   * @param qz target rotation z
   * @param qw target rotation w
   * @param dtSeconds duration of the next step
   */
  public static native void kinematicMoveTo(
      long world,
      int index,
      float xMeters,
      float yMeters,
      float zMeters,
      float qx,
      float qy,
      float qz,
      float qw,
      double dtSeconds);
}
