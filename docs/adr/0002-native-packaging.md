# ADR-0002: Native library packaging as WPILib JNI zips

- **Status:** Accepted
- **Date:** 2026-09-12

## Context

Robot projects consume third-party code through WPILib *vendordeps*: JSON files in `vendordeps/`
that GradleRIO turns into Maven dependencies. The native library must reach desktop simulation
(`simulateJava`) and desktop unit tests (`test`), and must never break deploys to the robot controller.

Behavior verified in `wpilibsuite/native-utils` (`WPIJavaVendorDepsExtension.jniInternal`):

- Every `jniDependencies` entry resolves to `group:artifactId:version:<platform>[debug]@zip`
  (`@jar` if `isJar` is true). The `debug` suffix is used when a project sets `wpi.java.debugJni = true`.
- An entry applies when `validPlatforms` contains the requested platform **and** its `simMode` allows
  the current sim mode (omitting `simMode` allows both `hwsim` and `swsim`). The robot-controller
  platform ignores `simMode`.
- If an entry does not apply and `skipInvalidPlatforms` is false, the build fails with
  `MissingVendorJniDependencyException`.

WPILib's own native zips (e.g. `wpiutil-cpp-2026.2.1-windowsx86-64.zip`) use the layout
`windows/x86-64/shared/<lib>`.

## Decision

1. Publish one Maven artifact **`frcsim-native`** as zips with classifiers
   `windowsx86-64`, `linuxx86-64`, `linuxarm64`, `osxuniversal`, plus a `…debug` twin of each.
2. Zip layout mirrors WPILib: `<os>/<arch>/shared/frcsim.{dll,so,dylib}` (the same tree
   `cmake --install` produces under `native/out/`).
3. **Debug classifiers contain the release binary.** A debug physics build is too slow to be useful, and
   publishing it would double CI time.
4. Vendordep `jniDependencies` entry: `isJar: false`, `skipInvalidPlatforms: true`, no `simMode`,
   desktop platforms only. Robot-controller builds skip it silently.
5. Java API ships as a normal jar `frcsim-java` in `javaDependencies`.
6. The vendordep is generated from `vendordep/frcsim.json.in` (`version`, `group`, `mavenUrl` substituted)
   by `:frcsim-native:generateVendordep`. Local end-to-end tests point `mavenUrl` at the file repo
   `java/build/repo`.
7. Windows DLLs link the **static MSVC runtime** (ADR-0001 / decision D3), so they don't depend on the
   JVM's runtime version.

## Consequences

- ✅ No custom extraction code: GradleRIO puts the library on `java.library.path`, and
  `System.loadLibrary("frcsim")` works.
- ✅ Deploying to the robot is unaffected.
- ❌ Every published platform needs a CI build; a platform listed in `validPlatforms` but not published
  fails dependency resolution on that host. CI release builds use `-Pfrcsim.requireAllPlatforms=true`.
- ❌ `wpi.java.debugJni = true` does not give a debuggable frcsim; documented.

## Verification

`examples/smoke-robot` is a stock WPILib 2026 Java template project whose unit tests load frcsim through
the generated vendordep:
```
cd java && ./gradlew :frcsim-native:installSmokeRobotVendordep
cd examples/smoke-robot && ./gradlew test
```
