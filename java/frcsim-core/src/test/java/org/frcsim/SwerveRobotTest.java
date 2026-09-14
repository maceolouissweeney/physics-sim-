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
      SwerveRobot.ROBOT_SIZE,
      SwerveRobot.YAW,
      SwerveRobot.QX,
      SwerveRobot.VX,
      SwerveRobot.WX,
      SwerveRobot.BATTERY_VOLTAGE,
      SwerveRobot.BROWNOUT,
      SwerveRobot.MODULE_COUNT,
      SwerveRobot.MODULES,
      SwerveRobot.MODULE_SIZE,
      SwerveRobot.DRIVE_ROTOR_POSITION,
      SwerveRobot.STEER_ANGLE,
      SwerveRobot.DRIVE_ROTOR_VELOCITY,
      SwerveRobot.STEER_VELOCITY,
      SwerveRobot.DRIVE_APPLIED_VOLTAGE,
      SwerveRobot.NORMAL_FORCE,
      SwerveRobot.SLIP_SPEED,
      SwerveRobot.GYRO_YAW,
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
      assertEquals(2.0, robot.x(), 1e-5);
      assertEquals(0.5, robot.yawRadians(), 1e-5);
      assertEquals(robot.yawRadians(), robot.gyroYawRadians(), 1e-12, "ideal gyro by default");

      run(world, 1.0);
      assertEquals(0.0, robot.z(), 0.003);
      double supported = 0;
      for (int m = 0; m < 4; m++) {
        supported += robot.normalForce(m);
      }
      assertEquals(60 * 9.80665, supported, 60 * 9.80665 * 0.03);

      for (int m = 0; m < 4; m++) {
        robot.setDriveVoltage(m, 6.0);
      }
      run(world, 1.0);
      double speed = Math.hypot(robot.vx(), robot.vy());
      assertTrue(speed > 1.5, "speed " + speed);
      assertEquals(0.5, Math.atan2(robot.vy(), robot.vx()), 0.05);
      assertTrue(robot.driveRotorPosition(0) > 10.0);
      assertTrue(robot.driveRotorVelocity(0) > 100.0);
      assertEquals(6.0, robot.driveAppliedVoltage(0), 1e-3);
      assertTrue(robot.batteryVoltage() < 12.5);
      assertFalse(robot.isBrownedOut());

      robot.resetPose(5, 5, 0);
      assertEquals(5.0, robot.x(), 1e-5);
      assertEquals(0.0, robot.vx(), 1e-5);
    }
  }

  @Test
  void steeringChangesModuleAngle() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveRobot robot =
          world.robots().addSwerve(SwerveDriveConfig.rectangular(0.55, 0.55), 2.0, 2.0, 0.0);
      for (int i = 0; i < 100; i++) {
        for (int m = 0; m < 4; m++) {
          double error = 1.0 - robot.steerAngle(m);
          robot.setSteerVoltage(m, Math.max(-12, Math.min(12, 20 * error)));
        }
        world.step(0.004, 1);
      }
      assertEquals(1.0, robot.steerAngle(2), 0.02);
      assertEquals(1.0 * 150.0 / 7.0, robot.steerRotorPosition(2), 0.5);
    }
  }

  @Test
  void configErrorsAndBounds() {
    SimWorld world = worldWithCarpet();
    try {
      SwerveDriveConfig tooFar = SwerveDriveConfig.rectangular(0.55, 0.55);
      tooFar.modules.get(0).x = 3.0;
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
      assertThrows(IndexOutOfBoundsException.class, () -> robot.setDriveVoltage(4, 1));
      world.close();
      assertThrows(IllegalStateException.class, robot::x);
    } finally {
      world.close(); // idempotent
    }
  }

  @Test
  void gyroDriftIsConfigurable() {
    try (SimWorld world = worldWithCarpet()) {
      SwerveDriveConfig config = SwerveDriveConfig.rectangular(0.55, 0.55);
      config.gyroYawDriftRate = 0.02;
      SwerveRobot robot = world.robots().addSwerve(config, 2, 2, 0);
      run(world, 5.0);
      assertEquals(0.1, robot.gyroYawRadians() - robot.yawRadians(), 1e-3);
    }
  }

  @Test
  void dualMotorPresetsScale() {
    DcMotorSpec one = DcMotorSpec.krakenX60(1);
    DcMotorSpec two = DcMotorSpec.krakenX60(2);
    assertEquals(one.stallTorque(), two.stallTorque());
    assertEquals(2, two.count());
    assertEquals(6000 * 2 * Math.PI / 60, one.freeSpeed(), 1e-9);
  }
}
