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
   * @param gravityZ gravity along +Z in m/s²
   * @param maxPieces game piece capacity
   * @param minVelocityForRestitution m/s below which impacts don't bounce
   * @param timeBeforeSleep seconds at rest before sleeping
   * @param sleepVelocityThreshold m/s considered at rest
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
      double gravityZ,
      int maxPieces,
      float minVelocityForRestitution,
      float timeBeforeSleep,
      float sleepVelocityThreshold,
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
   * @param height top surface height
   * @param material material id
   * @return field primitive index
   */
  public static native int fieldAddGround(long world, float height, int material);

  /**
   * Adds a static box.
   *
   * @param world native handle
   * @param name primitive name
   * @param cx center x
   * @param cy center y
   * @param cz center z
   * @param hx half extent x
   * @param hy half extent y
   * @param hz half extent z
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
      float cx,
      float cy,
      float cz,
      float hx,
      float hy,
      float hz,
      float qx,
      float qy,
      float qz,
      float qw,
      int material);

  /**
   * Sets field bounds.
   *
   * @param world native handle
   * @param minX min x
   * @param minY min y
   * @param minZ min z
   * @param maxX max x
   * @param maxY max y
   * @param maxZ max z
   */
  public static native void fieldSetBounds(
      long world, float minX, float minY, float minZ, float maxX, float maxY, float maxZ);

  // ---- Game pieces --------------------------------------------------------------------------

  /**
   * Registers a piece type.
   *
   * @param world native handle
   * @param name unique name
   * @param shape shape code
   * @param radius radius
   * @param halfHeight cylinder half height
   * @param hx box half extent x
   * @param hy box half extent y
   * @param hz box half extent z
   * @param mass mass in kg
   * @param material material id
   * @param maxAngularVelocity max angular velocity in rad/s
   * @return piece type id
   */
  public static native int pieceTypeAdd(
      long world,
      String name,
      int shape,
      float radius,
      float halfHeight,
      float hx,
      float hy,
      float hz,
      float mass,
      int material,
      float maxAngularVelocity);

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
   * @param positionsXyz xyz triples
   * @param velocitiesXyz xyz triples or null
   * @param outIndices receives indices, or null
   */
  public static native void piecesSpawn(
      long world, int type, float[] positionsXyz, float[] velocitiesXyz, int[] outIndices);

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
   * @param x position x
   * @param y position y
   * @param z position z
   * @param vx linear velocity x
   * @param vy linear velocity y
   * @param vz linear velocity z
   * @param wx angular velocity x
   * @param wy angular velocity y
   * @param wz angular velocity z
   */
  public static native void pieceTeleport(
      long world,
      int index,
      float x,
      float y,
      float z,
      float vx,
      float vy,
      float vz,
      float wx,
      float wy,
      float wz);

  /**
   * Direct buffer over piece positions (3 floats per index, native byte order).
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
   * @param x field x
   * @param y field y
   * @param yaw heading
   * @return robot index
   */
  public static native int robotAddSwerve(
      long world,
      float[] robotParams,
      int bumperMaterial,
      float[] moduleParams,
      float x,
      float y,
      float yaw);

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
   * @param x field x
   * @param y field y
   * @param yaw heading
   */
  public static native void robotResetPose(long world, int robot, float x, float y, float yaw);

  /**
   * Native robot I/O layout: {@code [robot size, yaw, qx, vx, wx, battery voltage, brownout, module
   * count, modules, module size, drive rotor position, steer angle, drive rotor velocity, steer
   * velocity, drive applied voltage, normal force, slip speed]}.
   *
   * @return layout array
   */
  public static native int[] robotIoLayout();

  // ---- Kinematic bodies ---------------------------------------------------------------------

  /**
   * Adds a kinematic box.
   *
   * @param world native handle
   * @param cx center x
   * @param cy center y
   * @param cz center z
   * @param hx half extent x
   * @param hy half extent y
   * @param hz half extent z
   * @param qx rotation quaternion x
   * @param qy rotation quaternion y
   * @param qz rotation quaternion z
   * @param qw rotation quaternion w
   * @param material material id
   * @return kinematic body index
   */
  public static native int kinematicAddBox(
      long world,
      float cx,
      float cy,
      float cz,
      float hx,
      float hy,
      float hz,
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
   * @param x target x
   * @param y target y
   * @param z target z
   * @param qx target rotation x
   * @param qy target rotation y
   * @param qz target rotation z
   * @param qw target rotation w
   * @param dtSeconds duration of the next step
   */
  public static native void kinematicMoveTo(
      long world,
      int index,
      float x,
      float y,
      float z,
      float qx,
      float qy,
      float qz,
      float qw,
      double dtSeconds);
}
