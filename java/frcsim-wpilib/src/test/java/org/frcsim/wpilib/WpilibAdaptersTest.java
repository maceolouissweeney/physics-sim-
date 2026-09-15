package org.frcsim.wpilib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import org.frcsim.DcMotorSpec;
import org.frcsim.Materials;
import org.frcsim.SimWorld;
import org.frcsim.SwerveDriveConfig;
import org.junit.jupiter.api.Test;

class WpilibAdaptersTest {
  /** Motor constants exactly as frcsim derives them natively: {R ohms, Kv rad/s/V, Kt N·m/A}. */
  private static double[] constants(DcMotorSpec s) {
    double stallCurrentAmps = s.stallCurrentAmps() * s.count();
    double freeCurrentAmps = s.freeCurrentAmps() * s.count();
    double resistanceOhms = s.nominalVolts() / stallCurrentAmps;
    double kvRadPerSecPerVolt =
        s.freeSpeedRadPerSec() / (s.nominalVolts() - resistanceOhms * freeCurrentAmps);
    double ktNewtonMetersPerAmp = s.stallTorqueNewtonMeters() * s.count() / stallCurrentAmps;
    return new double[] {resistanceOhms, kvRadPerSecPerVolt, ktNewtonMetersPerAmp};
  }

  @Test
  void dcMotorConversionPreservesElectricalConstants() {
    DcMotorSpec converted = WpilibMotors.fromDCMotor(DCMotor.getKrakenX60Foc(2), 1.2e-4);
    DcMotorSpec preset = DcMotorSpec.krakenX60Foc(2);
    double[] a = constants(converted);
    double[] b = constants(preset);
    for (int i = 0; i < 3; i++) {
      assertEquals(b[i], a[i], Math.abs(b[i]) * 1e-6);
    }
    DCMotor wpilib = DCMotor.getKrakenX60Foc(2);
    assertEquals(wpilib.rOhms, a[0], 1e-9);
    assertEquals(wpilib.KvRadPerSecPerVolt, a[1], 1e-6);
    assertEquals(wpilib.KtNMPerAmp, a[2], 1e-9);
  }

  @Test
  void swerveDriveViewIsConsistentWithGroundTruth() {
    try (SimWorld world = SimWorld.create()) {
      world.field().addGround(0, world.materials().get(Materials.CARPET));
      Pose2d start = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
      SimSwerveDrive drive =
          SimSwerveDrive.create(world, SwerveDriveConfig.rectangular(0.55, 0.55), start);
      assertEquals(start.getX(), drive.getPose().getX(), 1e-5);
      assertEquals(30, drive.getGyroYaw().getDegrees(), 1e-3);
      assertEquals(4, drive.getModuleTranslations().length);

      for (int i = 0; i < 25; i++) {
        world.step(0.020);
      }
      for (int m = 0; m < 4; m++) {
        drive.setModuleCommandVolts(m, 4.0, 0.0);
      }
      for (int i = 0; i < 25; i++) {
        world.step(0.020);
      }

      // Cruise for 0.5 s and compare encoder odometry with ground truth.
      Pose2d before = drive.getPose();
      SwerveModulePosition[] p0 = drive.getModulePositions();
      for (int i = 0; i < 25; i++) {
        world.step(0.020);
      }
      Pose2d after = drive.getPose();
      SwerveModulePosition[] p1 = drive.getModulePositions();
      double traveledMeters = after.getTranslation().getDistance(before.getTranslation());
      double encoderDistanceMeters = p1[0].distanceMeters - p0[0].distanceMeters;
      assertTrue(traveledMeters > 0.5, "traveled " + traveledMeters);
      assertEquals(traveledMeters, encoderDistanceMeters, traveledMeters * 0.05);

      ChassisSpeeds robotRelative = drive.getRobotRelativeSpeeds();
      SwerveModuleState[] states = drive.getModuleStates();
      assertEquals(states[0].speedMetersPerSecond, robotRelative.vxMetersPerSecond, 0.05);
      assertEquals(0.0, robotRelative.vyMetersPerSecond, 0.05);
      assertEquals(0.0, states[0].angle.getRadians(), 0.02);
      assertEquals(
          Math.toRadians(30),
          Math.atan2(
              drive.getFieldRelativeSpeeds().vyMetersPerSecond,
              drive.getFieldRelativeSpeeds().vxMetersPerSecond),
          0.03);

      drive.resetPose(new Pose2d(8, 4, Rotation2d.kZero));
      assertEquals(8.0, drive.getPose().getX(), 1e-5);
      assertEquals(0.0, drive.getPose3d().getZ(), 1e-3);
    }
  }
}
