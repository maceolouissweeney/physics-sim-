package org.frcsim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.frcsim.jni.FrcSimJNI;
import org.junit.jupiter.api.Test;

/** Swerve robots through the Java API (JNI + shared I/O block). */
class SwerveRobotTest {
  private static final double GRAVITY_METERS_PER_SEC_SQ =
      WorldConfig.STANDARD_GRAVITY_METERS_PER_SEC_SQ;

  private static SimWorld worldWithCarpet() {
    SimWorld world = SimWorld.create();
    world.field().addGround(0, world.materials().get(Materials.CARPET));
    return world;
  }

  private static void run(SimWorld world, double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.020); i++) {
      world.step(0.020);
    }
  }

  @Test
  void ioLayoutMatchesNative() {
    FrcSim.ensureLoaded();
    int[] expected = {
      SwerveRobot.ROBOT_SIZE_BYTES,
      SwerveRobot.YAW_RADIANS,
      SwerveRobot.QX,
      SwerveRobot.VX_METERS_PER_SEC,
      SwerveRobot.WX_RAD_PER_SEC,
      SwerveRobot.BATTERY_VOLTS,
      SwerveRobot.BROWNOUT,
      SwerveRobot.MODULE_COUNT,
      SwerveRobot.MODULES,
      SwerveRobot.MODULE_SIZE_BYTES,
      SwerveRobot.DRIVE_ROTOR_POSITION_RADIANS,
      SwerveRobot.STEER_ANGLE_RADIANS,
      SwerveRobot.DRIVE_ROTOR_VELOCITY_RAD_PER_SEC,
      SwerveRobot.STEER_VELOCITY_RAD_PER_SEC,
      SwerveRobot.DRIVE_APPLIED_VOLTS,
      SwerveRobot.NORMAL_FORCE_NEWTONS,
      SwerveRobot.SLIP_SPEED_METERS_PER_SEC,
      SwerveRobot.GYRO_YAW_RADIANS,
    };
    assertArrayEquals(expected, FrcSimJNI.robotIoLayout());
  }

  @Test
  void drivesAlongHeadingAndReportsSensors() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveRobot robot =
          world.robots().addSwerve(SwerveDriveConfig.rectangular(0.55, 0.55), 2.0, 3.0, 0.5);
      assertEquals(4, robot.moduleCount());
      assertSame(robot, world.robots().swerve(0));
      assertEquals(2.0, robot.xMeters(), 1e-5);
      assertEquals(0.5, robot.yawRadians(), 1e-5);
      assertEquals(robot.yawRadians(), robot.gyroYawRadians(), 1e-12, "ideal gyro by default");

      run(world, 1.0);
      assertEquals(0.0, robot.zMeters(), 0.003);
      double supportedNewtons = 0;
      for (int m = 0; m < 4; m++) {
        supportedNewtons += robot.normalForceNewtons(m);
      }
      assertEquals(
          60 * GRAVITY_METERS_PER_SEC_SQ, supportedNewtons, 60 * GRAVITY_METERS_PER_SEC_SQ * 0.03);

      for (int m = 0; m < 4; m++) {
        robot.setDriveCommandVolts(m, 6.0);
      }
      run(world, 1.0);
      double speedMetersPerSec = Math.hypot(robot.vxMetersPerSec(), robot.vyMetersPerSec());
      assertTrue(speedMetersPerSec > 1.5, "speed " + speedMetersPerSec);
      assertEquals(0.5, Math.atan2(robot.vyMetersPerSec(), robot.vxMetersPerSec()), 0.05);
      assertTrue(robot.driveRotorPositionRadians(0) > 10.0);
      assertTrue(robot.driveRotorVelocityRadPerSec(0) > 100.0);
      assertEquals(6.0, robot.driveAppliedVolts(0), 1e-3);
      assertTrue(robot.batteryVolts() < 12.5);
      assertFalse(robot.isBrownedOut());

      robot.resetPose(5, 5, 0);
      assertEquals(5.0, robot.xMeters(), 1e-5);
      assertEquals(0.0, robot.vxMetersPerSec(), 1e-5);
    }
  }

  @Test
  void steeringChangesModuleAngle() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveRobot robot =
          world.robots().addSwerve(SwerveDriveConfig.rectangular(0.55, 0.55), 2.0, 2.0, 0.0);
      for (int i = 0; i < 100; i++) {
        for (int m = 0; m < 4; m++) {
          double errorRadians = 1.0 - robot.steerAngleRadians(m);
          robot.setSteerCommandVolts(m, Math.max(-12, Math.min(12, 20 * errorRadians)));
        }
        world.step(0.004, 1);
      }
      assertEquals(1.0, robot.steerAngleRadians(2), 0.02);
      assertEquals(1.0 * 150.0 / 7.0, robot.steerRotorPositionRadians(2), 0.5);
    }
  }

  @Test
  void couplingRatioFeedsDriveRotorPosition() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveModuleConfig module = new SwerveModuleConfig();
      module.couplingGearRatio = 50.0 / 14.0;
      SwerveRobot robot =
          world
              .robots()
              .addSwerve(SwerveDriveConfig.rectangular(0.55, 0.55, module), 2.0, 2.0, 0.0);
      for (int m = 0; m < 4; m++) {
        robot.setSteerCommandVolts(m, 2.0);
      }
      run(world, 0.2);
      assertTrue(Math.abs(robot.steerAngleRadians(0)) > 0.1);
      assertTrue(
          Math.abs(robot.driveRotorPositionRadians(0)) > 0.3,
          "steering alone moves the drive rotor through the coupling gear");
    }
  }

  @Test
  void configErrorsAndBounds() {
    SimWorld world = worldWithCarpet();
    try {
      SwerveDriveConfig tooFar = SwerveDriveConfig.rectangular(0.55, 0.55);
      tooFar.modules.get(0).xMeters = 3.0;
      IllegalArgumentException e =
          assertThrows(
              IllegalArgumentException.class, () -> world.robots().addSwerve(tooFar, 0, 0, 0));
      assertTrue(e.getMessage().contains("modules[0]"), e.getMessage());

      SwerveDriveConfig none = SwerveDriveConfig.rectangular(0.55, 0.55);
      none.modules.clear();
      assertThrows(IllegalArgumentException.class, () -> world.robots().addSwerve(none, 0, 0, 0));

      SwerveRobot robot =
          world.robots().addSwerve(SwerveDriveConfig.rectangular(0.55, 0.55), 1, 1, 0);
      assertEquals(0, robot.index(), "failed adds do not consume indices");
      assertThrows(IndexOutOfBoundsException.class, () -> robot.setDriveCommandVolts(4, 1));
      world.close();
      assertThrows(IllegalStateException.class, robot::xMeters);
    } finally {
      world.close(); // idempotent
    }
  }

  @Test
  void gyroDriftIsConfigurable() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveDriveConfig config = SwerveDriveConfig.rectangular(0.55, 0.55);
      config.gyroYawDriftRateRadPerSec = 0.02;
      SwerveRobot robot = world.robots().addSwerve(config, 2, 2, 0);
      run(world, 5.0);
      assertEquals(0.1, robot.gyroYawRadians() - robot.yawRadians(), 1e-3);
    }
  }

  @Test
  void motorPresetsScaleAndIncludeMinion() {
    DcMotorSpec one = DcMotorSpec.krakenX60(1);
    DcMotorSpec two = DcMotorSpec.krakenX60(2);
    assertEquals(one.stallTorqueNewtonMeters(), two.stallTorqueNewtonMeters());
    assertEquals(2, two.count());
    assertEquals(6000 * 2 * Math.PI / 60, one.freeSpeedRadPerSec(), 1e-9);
    assertEquals(7704 * 2 * Math.PI / 60, DcMotorSpec.minion(1).freeSpeedRadPerSec(), 1e-9);
    assertEquals(3, one.withCount(3).count());
  }
}
