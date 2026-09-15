package org.frcsim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/** Pure-Java validation tests; these do not load the native library. */
class WorldConfigTest {
  @Test
  void defaultsAreValid() {
    WorldConfig config = WorldConfig.defaults();
    assertEquals(2, config.workerThreads());
    assertEquals(5, config.substeps());
    assertEquals(1024, config.maxPieces());
    assertEquals(WorldConfig.STANDARD_GRAVITY_METERS_PER_SEC_SQ, config.gravityMetersPerSecSq());
    assertEquals(0.2, config.minVelocityForRestitutionMetersPerSec());
    assertEquals(0.5, config.timeBeforeSleepSeconds());
    assertEquals(0.03, config.sleepVelocityThresholdMetersPerSec());
  }

  @Test
  void toBuilderRoundTripsAndModifies() {
    WorldConfig base = WorldConfig.defaults();
    WorldConfig modified =
        base.toBuilder().workerThreads(0).gravityMetersPerSecSq(0.0).substeps(10).build();
    assertEquals(0, modified.workerThreads());
    assertEquals(0.0, modified.gravityMetersPerSecSq());
    assertEquals(10, modified.substeps());
    assertEquals(base.maxPieces(), modified.maxPieces());
    assertEquals(2, base.workerThreads());
  }

  @Test
  void rejectsInvalidValues() {
    assertThrows(IllegalArgumentException.class, () -> WorldConfig.builder().maxBodies(0).build());
    assertThrows(
        IllegalArgumentException.class, () -> WorldConfig.builder().workerThreads(-2).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> WorldConfig.builder().workerThreads(WorldConfig.MAX_WORKER_THREADS + 1).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> WorldConfig.builder().gravityMetersPerSecSq(-1.0).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> WorldConfig.builder().gravityMetersPerSecSq(Double.NaN).build());
    assertThrows(IllegalArgumentException.class, () -> WorldConfig.builder().substeps(0).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> WorldConfig.builder().maxBodies(100).maxPieces(200).build());
    assertThrows(
        IllegalArgumentException.class, () -> WorldConfig.builder().solverVelocitySteps(0).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> WorldConfig.builder().sleepVelocityThresholdMetersPerSec(-0.1).build());
  }
}
