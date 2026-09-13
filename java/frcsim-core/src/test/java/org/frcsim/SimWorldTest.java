package org.frcsim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.frcsim.jni.FrcSimJNI;
import org.junit.jupiter.api.Test;

/** Integration tests that exercise the real native library through JNI. */
class SimWorldTest {
  @Test
  void nativeVersionMatchesBuildVersion() {
    String expected = System.getProperty("frcsim.expectedVersion");
    assertEquals(expected, FrcSim.nativeVersion());
    assertEquals(1, FrcSim.nativeAbiVersion());
  }

  @Test
  void statsLayoutMatchesNative() {
    FrcSim.ensureLoaded();
    int[] expected = {
      WorldStats.SIZE,
      WorldStats.OFFSET_TIME_SECONDS,
      WorldStats.OFFSET_LAST_STEP_WALL_SECONDS,
      WorldStats.OFFSET_SUBSTEP_COUNT,
      WorldStats.OFFSET_ACTIVE_BODIES,
      WorldStats.OFFSET_UPDATE_ERROR_FLAGS,
      WorldStats.OFFSET_PIECE_HIGH_WATER,
      WorldStats.OFFSET_PIECES_SIMULATED,
    };
    assertArrayEquals(expected, FrcSimJNI.worldStatsLayout());
  }

  @Test
  void stepsAdvanceTimeAndStats() {
    try (SimWorld world = SimWorld.create()) {
      for (int i = 0; i < 50; i++) {
        world.step(0.020);
      }
      assertEquals(1.0, world.timeSeconds(), 1e-12);
      assertEquals(250, world.stats().substepCount());
      assertTrue(world.stats().lastStepWallSeconds() > 0.0);
      assertEquals(0, world.stats().updateErrorFlags());
    }
  }

  @Test
  void customConfigWithThreadPool() {
    WorldConfig config = WorldConfig.builder().workerThreads(2).maxPieces(16).build();
    try (SimWorld world = SimWorld.create(config)) {
      world.step(0.020, 10);
      assertEquals(0.020, world.timeSeconds(), 1e-12);
      assertEquals(16, world.pieces().capacity());
    }
  }

  @Test
  void invalidStepArgumentsThrowIllegalArgument() {
    try (SimWorld world = SimWorld.create()) {
      IllegalArgumentException e =
          assertThrows(IllegalArgumentException.class, () -> world.step(-1.0, 5));
      assertTrue(e.getMessage().contains("dt"), e.getMessage());
      assertThrows(IllegalArgumentException.class, () -> world.step(0.02, 0));
      assertEquals(0.0, world.timeSeconds());
    }
  }

  @Test
  void closeIsIdempotentAndViewsThrowAfterClose() {
    SimWorld world = SimWorld.create();
    GamePieces pieces = world.pieces();
    WorldStats stats = world.stats();
    assertFalse(world.isClosed());
    world.close();
    assertTrue(world.isClosed());
    assertDoesNotThrow(world::close);
    assertThrows(IllegalStateException.class, () -> world.step(0.02));
    assertThrows(IllegalStateException.class, world::timeSeconds);
    assertThrows(IllegalStateException.class, () -> pieces.x(0));
    assertThrows(IllegalStateException.class, stats::activeBodies);
    assertThrows(IllegalStateException.class, () -> world.materials().find("carpet"));
  }

  @Test
  void multipleWorldsCoexist() {
    try (SimWorld a = SimWorld.create();
        SimWorld b = SimWorld.create()) {
      a.step(0.02);
      assertEquals(0.02, a.timeSeconds(), 1e-12);
      assertEquals(0.0, b.timeSeconds());
    }
  }
}
