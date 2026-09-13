package org.frcsim;

import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.Optional;
import org.frcsim.jni.FrcSimJNI;

/**
 * Material registry of a {@link SimWorld}. Every world starts with {@value #DEFAULT} and the
 * standard FRC materials below, whose values are estimates pending calibration.
 *
 * <p>When two materials touch, friction combines as {@code sqrt(a * b)} and restitution as {@code
 * max(a, b)}, unless overridden with {@link #setPair}.
 */
public final class Materials {
  /** Always present, id 0. */
  public static final String DEFAULT = "default";

  /** FRC field carpet. */
  public static final String CARPET = "carpet";

  /** Polycarbonate field walls and panels. */
  public static final String POLYCARBONATE = "polycarbonate";

  /** Aluminum field structure. */
  public static final String ALUMINUM = "aluminum";

  /** Robot bumpers. */
  public static final String BUMPER = "bumper";

  /** High-density foam (e.g. REBUILT fuel). */
  public static final String FOAM = "foam";

  private final SimWorld world;

  Materials(SimWorld world) {
    this.world = world;
  }

  /**
   * Adds a material, or updates it if the name already exists.
   *
   * @param name material name
   * @param friction friction coefficient, &gt;= 0
   * @param restitution restitution coefficient, 0..1
   * @return material handle
   * @throws IllegalArgumentException for invalid values
   * @throws CapacityExceededException if 64 materials already exist
   */
  public Material add(String name, double friction, double restitution) {
    Objects.requireNonNull(name, "name");
    int id =
        FrcSimJNI.materialAdd(world.nativeHandle(), name, (float) friction, (float) restitution);
    return new Material(id, name);
  }

  /**
   * Looks up a material by name.
   *
   * @param name material name
   * @return material handle, or empty if not registered
   */
  public Optional<Material> find(String name) {
    Objects.requireNonNull(name, "name");
    int id = FrcSimJNI.materialFind(world.nativeHandle(), name);
    return id < 0 ? Optional.empty() : Optional.of(new Material(id, name));
  }

  /**
   * Looks up a material that must exist.
   *
   * @param name material name
   * @return material handle
   * @throws NoSuchElementException if not registered
   */
  public Material get(String name) {
    return find(name)
        .orElseThrow(() -> new NoSuchElementException("unknown material '" + name + "'"));
  }

  /**
   * Overrides the combined friction and restitution used when two materials touch.
   *
   * @param a first material
   * @param b second material
   * @param friction combined friction
   * @param restitution combined restitution
   */
  public void setPair(Material a, Material b, double friction, double restitution) {
    Objects.requireNonNull(a, "a");
    Objects.requireNonNull(b, "b");
    FrcSimJNI.materialSetPair(
        world.nativeHandle(), a.id(), b.id(), (float) friction, (float) restitution);
  }
}
