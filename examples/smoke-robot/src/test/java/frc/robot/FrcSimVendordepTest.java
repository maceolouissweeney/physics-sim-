package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.frcsim.FrcSim;
import org.frcsim.SimWorld;
import org.junit.jupiter.api.Test;

/** Verifies the vendordep delivers a loadable native library to GradleRIO desktop tests. */
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
}
