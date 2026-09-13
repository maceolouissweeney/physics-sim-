package org.frcsim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.util.NoSuchElementException;
import org.junit.jupiter.api.Test;

/** Materials, field loading, game pieces, and kinematic bodies through the Java API. */
class FieldAndPiecesTest {
  private static final double FUEL_RADIUS = 0.075;

  private static Path testField() {
    return Path.of(System.getProperty("frcsim.repoDir"), "fields", "test-flat", "field.json");
  }

  private static void run(SimWorld world, int periods) {
    for (int i = 0; i < periods; i++) {
      world.step(0.020);
    }
  }

  @Test
  void loadsFieldAndSettlesPieces() throws IOException {
    try (SimWorld world = SimWorld.create()) {
      world.field().loadJson(testField());
      GamePieceType fuel = world.pieces().findType("fuel").orElseThrow();

      int[] indices = world.pieces().spawn(fuel, new float[] {2, 2, 1, 3, 3, 1, 4, 4, 1});
      assertEquals(3, indices.length);
      assertEquals(3, world.pieces().count());
      run(world, 100);

      assertEquals(FUEL_RADIUS, world.pieces().z(indices[1]), 0.005);
      assertEquals(3.0, world.pieces().x(indices[1]), 0.01);
      assertEquals(PieceState.ON_FIELD, world.pieces().state(indices[1]));
      assertEquals(3, world.stats().piecesSimulated());
      assertEquals(3, world.pieces().countInState(PieceState.ON_FIELD));

      float[] copy = new float[9];
      assertEquals(3, world.pieces().copyPositions(copy));
      assertEquals(4.0f, copy[6], 0.01f);
    }
  }

  @Test
  void pieceLifecycle() throws IOException {
    try (SimWorld world = SimWorld.create()) {
      world.field().loadJson(testField());
      GamePieceType fuel = world.pieces().findType("fuel").orElseThrow();
      int piece = world.pieces().spawn(fuel, 5, 5, FUEL_RADIUS);

      world.pieces().setState(piece, PieceState.IN_ROBOT);
      assertEquals(PieceState.IN_ROBOT, world.pieces().state(piece));
      assertEquals(0, world.stats().piecesSimulated());

      world.pieces().teleport(piece, 6, 5, 0.5, 1, 0, 0);
      assertEquals(PieceState.ON_FIELD, world.pieces().state(piece));

      world.pieces().despawn(piece);
      assertEquals(PieceState.INACTIVE, world.pieces().state(piece));
      assertEquals(piece, world.pieces().spawn(fuel, 1, 1, 1), "index is recycled");

      assertThrows(
          IllegalArgumentException.class,
          () -> world.pieces().setState(piece, PieceState.INACTIVE));
      assertThrows(NoSuchElementException.class, () -> world.pieces().despawn(900));
      assertThrows(IndexOutOfBoundsException.class, () -> world.pieces().x(-1));
    }
  }

  @Test
  void piecesLeavingBoundsBecomeOutOfBounds() {
    try (SimWorld world = SimWorld.create()) {
      Material carpet = world.materials().get(Materials.CARPET);
      world.field().addGround(0, carpet);
      world.field().setBounds(-1, -1, -1, 1, 1, 5);
      GamePieceType ball =
          world.pieces().registerType(GamePieceTypeSpec.sphere("ball", 0.1, 0.3, carpet));
      int piece = world.pieces().spawn(ball, 0, 0, 0.1);
      world.pieces().teleport(piece, 0, 0, 0.1, 5, 0, 0);
      run(world, 50);
      assertEquals(PieceState.OUT_OF_BOUNDS, world.pieces().state(piece));
    }
  }

  @Test
  void capacityIsEnforcedAllOrNothing() {
    try (SimWorld world = SimWorld.create(WorldConfig.builder().maxPieces(4).build())) {
      GamePieceType cube =
          world
              .pieces()
              .registerType(
                  GamePieceTypeSpec.box(
                      "cube", 0.12, 0.12, 0.12, 0.3, world.materials().get("default")));
      assertThrows(
          CapacityExceededException.class, () -> world.pieces().spawn(cube, new float[15]));
      assertEquals(0, world.pieces().count());
    }
  }

  @Test
  void materialsRegistry() {
    try (SimWorld world = SimWorld.create()) {
      Material ice = world.materials().add("ice", 0.05, 0.1);
      Material carpet = world.materials().get(Materials.CARPET);
      assertNotEquals(ice.id(), carpet.id());
      assertEquals(ice, world.materials().find("ice").orElseThrow());
      assertTrue(world.materials().find("unobtainium").isEmpty());
      world.materials().setPair(ice, carpet, 0.2, 0.0);
      assertThrows(IllegalArgumentException.class, () -> world.materials().add("bad", -1, 0));
      assertThrows(NoSuchElementException.class, () -> world.materials().get("nope"));
    }
  }

  @Test
  void invalidFieldJsonReportsLocation() {
    try (SimWorld world = SimWorld.create()) {
      IllegalArgumentException e =
          assertThrows(
              IllegalArgumentException.class,
              () ->
                  world
                      .field()
                      .loadJson(
                          "{\"schema\": \"frcsim.field/1\", \"statics\": [{\"type\": \"box\"}]}"));
      assertTrue(e.getMessage().contains("statics[0]"), e.getMessage());
    }
  }

  @Test
  void kinematicPlowPushesPiece() {
    try (SimWorld world = SimWorld.create()) {
      Material carpet = world.materials().get(Materials.CARPET);
      world.field().addGround(0, carpet);
      GamePieceType fuel =
          world
              .pieces()
              .registerType(
                  GamePieceTypeSpec.sphere(
                      "fuel", FUEL_RADIUS, 0.215, world.materials().get(Materials.FOAM)));
      int piece = world.pieces().spawn(fuel, 1.0, 0, FUEL_RADIUS);
      int plow =
          world
              .kinematics()
              .addBox(0, 0, 0.1, 0.45, 0.45, 0.08, world.materials().get(Materials.BUMPER));
      for (int i = 1; i <= 50; i++) {
        world.kinematics().moveTo(plow, 0.03 * i, 0, 0.1, 0, 0.020);
        world.step(0.020);
      }
      assertTrue(world.pieces().x(piece) > 1.5 + 0.45, "piece x = " + world.pieces().x(piece));
    }
  }
}
