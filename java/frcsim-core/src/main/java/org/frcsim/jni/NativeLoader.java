package org.frcsim.jni;

import org.frcsim.FrcSimException;

/**
 * Loads the frcsim native library and verifies its ABI version. Internal; use {@link
 * org.frcsim.FrcSim#ensureLoaded()}.
 *
 * <p>Resolution order:
 *
 * <ol>
 *   <li>System property {@value #LIBRARY_PATH_PROPERTY}: absolute path to the library file.
 *   <li>{@code System.loadLibrary("frcsim")} using {@code java.library.path}. GradleRIO sets this
 *       up for desktop simulation and unit tests when the vendordep is installed.
 * </ol>
 */
public final class NativeLoader {
  /** Base library name ({@code frcsim.dll}, {@code libfrcsim.so}, {@code libfrcsim.dylib}). */
  public static final String LIBRARY_NAME = "frcsim";

  /** System property that overrides library resolution with an absolute file path. */
  public static final String LIBRARY_PATH_PROPERTY = "frcsim.native.library";

  /** Must equal {@code FRCSIM_ABI_VERSION} in {@code frcsim_c.h}. */
  public static final int EXPECTED_ABI_VERSION = 1;

  private static boolean loaded;

  private NativeLoader() {}

  /**
   * Loads the library once per process.
   *
   * @throws FrcSimException if loading fails or the ABI version does not match
   */
  public static synchronized void ensureLoaded() {
    if (loaded) {
      return;
    }
    String explicitPath = System.getProperty(LIBRARY_PATH_PROPERTY);
    try {
      if (explicitPath != null && !explicitPath.isBlank()) {
        System.load(explicitPath);
      } else {
        System.loadLibrary(LIBRARY_NAME);
      }
    } catch (UnsatisfiedLinkError e) {
      throw new FrcSimException(loadFailureMessage(explicitPath, e), e);
    }

    int abi = FrcSimJNI.abiVersion();
    if (abi != EXPECTED_ABI_VERSION) {
      throw new FrcSimException(
          "frcsim native library ABI version "
              + abi
              + " does not match the Java binding (expected "
              + EXPECTED_ABI_VERSION
              + "). The frcsim-java and frcsim-native artifacts must have the same version.");
    }
    loaded = true;
  }

  private static String loadFailureMessage(String explicitPath, UnsatisfiedLinkError error) {
    StringBuilder msg = new StringBuilder(512);
    msg.append("Failed to load the frcsim native library.\n");
    if (explicitPath != null && !explicitPath.isBlank()) {
      msg.append("  ")
          .append(LIBRARY_PATH_PROPERTY)
          .append(" = ")
          .append(explicitPath)
          .append('\n');
    } else {
      msg.append("  java.library.path = ")
          .append(System.getProperty("java.library.path"))
          .append('\n');
    }
    msg.append("  platform = ")
        .append(System.getProperty("os.name"))
        .append(' ')
        .append(System.getProperty("os.arch"))
        .append('\n')
        .append("  cause = ")
        .append(error.getMessage())
        .append('\n')
        .append(
            "frcsim runs only in desktop simulation (Windows x86-64, Linux x86-64/arm64, macOS).\n")
        .append("Robot projects: make sure the frcsim vendordep is installed.\n")
        .append("Library development: build and install the native core (docs/building.md).");
    return msg.toString();
  }
}
