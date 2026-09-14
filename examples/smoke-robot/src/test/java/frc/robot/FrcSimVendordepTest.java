package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.frcsim.FrcSim;
import org.frcsim.Materials;
import org.frcsim.SimWorld;
import org.frcsim.SwerveDriveConfig;
import org.frcsim.SwerveModuleConfig;
import org.frcsim.wpilib.SimSwerveDrive;
import org.frcsim.wpilib.WpilibMotors;
import org.junit.jupiter.api.Test;

/** Verifies the vendordep delivers a loadable native library and the WPILib adapters to GradleRIO tests. */
class FrcSimVendordepTest {
  @Test
  void nativeLibraryLoadsThroughVendordep() {
    assertFalse(FrcSim.nativeVersion().isEmpty());
  }

  @Test
  void worldStepsInRobotProject() {
    try (SimWorld world = SimWorld.create()) {
      for (int i = 0; i < 50; i++) {
        world.step(0.020);
      }
      assertEquals(1.0, world.timeSeconds(), 1e-12);
    }
  }

  @Test
  void swerveDriveThroughWpilibAdapters() {
    try (SimWorld world = SimWorld.create()) {
      world.field().addGround(0, world.materials().get(Materials.CARPET));
      SwerveModuleConfig module = new SwerveModuleConfig();
      module.driveMotor = WpilibMotors.fromDCMotor(DCMotor.getKrakenX60Foc(1), 6.0e-5);
      SimSwerveDrive drive =
          SimSwerveDrive.create(
              world,
              SwerveDriveConfig.rectangular(0.55, 0.55, module),
              new Pose2d(1, 1, Rotation2d.kZero));

      for (int m = 0; m < 4; m++) {
        drive.setModuleVoltages(m, 6.0, 0.0);
      }
      for (int i = 0; i < 50; i++) {
        world.step(0.020);
      }
      assertTrue(drive.getPose().getX() > 1.5, "pose " + drive.getPose());
      assertEquals(4, drive.getModulePositions().length);
      assertTrue(drive.getBatteryVoltage() < 12.5);
    }
  }
}
