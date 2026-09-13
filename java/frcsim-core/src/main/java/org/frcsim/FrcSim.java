package org.frcsim;

import org.frcsim.jni.FrcSimJNI;
import org.frcsim.jni.NativeLoader;

/** Library-level entry points: native loading and version information. */
public final class FrcSim {
  private FrcSim() {}

  /**
   * Loads the native library if it has not been loaded yet. Other API entry points call this
   * automatically; call it explicitly to surface load failures early (for example at robot init).
   *
   * @throws FrcSimException if the library cannot be loaded or its ABI version does not match
   */
  public static void ensureLoaded() {
    NativeLoader.ensureLoaded();
  }

  /**
   * Returns the version of the loaded native library, for example {@code "0.1.0"}.
   *
   * @return native library version string
   */
  public static String nativeVersion() {
    NativeLoader.ensureLoaded();
    return FrcSimJNI.versionString();
  }

  /**
   * Returns the C ABI version of the loaded native library.
   *
   * @return native ABI version
   */
  public static int nativeAbiVersion() {
    NativeLoader.ensureLoaded();
    return FrcSimJNI.abiVersion();
  }
}
