# Building frcsim

frcsim has two halves:

- `native/`: the C++20 simulation core, built with CMake into one shared library (`frcsim.dll` /
  `libfrcsim.so` / `libfrcsim.dylib`) that exports the C ABI and the JNI entry points.
- `java/`: the Gradle build for the Java API. Its tests load the native library you just built.

The Java tests need the native library, so build and install the native side first.

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| C++ compiler | MSVC 2022 (Windows), GCC ≥ 12 or Clang ≥ 16 (Linux), Xcode clang (macOS) | Windows: *Visual Studio 2022 Build Tools* with the "Desktop development with C++" workload |
| CMake | ≥ 3.25 | |
| Ninja | any recent | Linux/macOS presets; optional on Windows |
| JDK | 17 | WPILib 2026 ships one: `C:\Users\Public\wpilib\2026\jdk`. Needed for JNI headers and Gradle. |
| Git + internet | | First configure downloads pinned dependencies (Jolt, GoogleTest) |

Windows install used during development:
```powershell
winget install Kitware.CMake
winget install Ninja-build.Ninja
winget install Microsoft.VisualStudio.2022.BuildTools --override "--passive --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```

## Native build

`JAVA_HOME` must point at a JDK so CMake can find the JNI headers. Run from `native/`.

### Windows (Visual Studio generator, no Developer Prompt needed)
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
cmake --preset windows-msvc                     # configure (downloads dependencies the first time)
cmake --build --preset windows-msvc-release     # build Release
ctest --preset windows-msvc-release             # run C++ tests
cmake --install build/windows-msvc --config Release   # -> native/out/windows/x86-64/shared/frcsim.dll
```

### Linux
```bash
export JAVA_HOME=/path/to/jdk17
cmake --preset ninja-release
cmake --build --preset ninja-release
ctest --preset ninja-release
cmake --install build/ninja-release             # -> native/out/linux/x86-64/shared/libfrcsim.so
```
Sanitizer build: `cmake --preset linux-asan && cmake --build --preset linux-asan && ctest --preset linux-asan`.

### macOS
Build each architecture slice, then combine them (decision D12):
```bash
cmake --preset macos-arm64  && cmake --build --preset macos-arm64
cmake --preset macos-x86_64 && cmake --build --preset macos-x86_64
mkdir -p out/osx/universal/shared
lipo -create build/macos-arm64/bin/libfrcsim.dylib build/macos-x86_64/bin/libfrcsim.dylib \
     -output out/osx/universal/shared/libfrcsim.dylib
```
For local Java tests you can also just `cmake --install build/macos-arm64` (installs to `out/osx/arm64/shared`).

### CMake options

| Option | Default | Meaning |
|---|---|---|
| `FRCSIM_BUILD_TESTS` | ON | GoogleTest executables `frcsim_core_tests`, `frcsim_capi_tests` |
| `FRCSIM_BUILD_JNI` | ON | Compile JNI glue into the shared library (needs `JAVA_HOME`) |
| `FRCSIM_BUILD_BENCH` | OFF | Benchmarks and scenario runner (Phase 1) |
| `FRCSIM_WARNINGS_AS_ERRORS` | OFF | ON in CI |
| `FRCSIM_DETERMINISTIC` | OFF | Jolt cross-platform determinism (slower; for regression tests) |
| `FRCSIM_SANITIZE` | "" | e.g. `address,undefined` (GCC/Clang) |

### Offline or custom dependency sources
Dependencies are pinned archives with SHA-256 hashes (`native/cmake/FrcsimDependencies.cmake`). To use a
local checkout instead:
```
cmake --preset windows-msvc -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=C:/src/JoltPhysics
```

## Java build

Run from `java/` after installing the native library.
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
./gradlew build          # compile, unit + JNI integration tests, spotlessCheck, javadoc
./gradlew spotlessApply  # auto-format Java sources
```
Tests read the native library from `native/out/<os>/<arch>/shared`. Point elsewhere with
`-Pfrcsim.nativeInstallRoot=<dir>`. At runtime you can also bypass `java.library.path` entirely with
`-Dfrcsim.native.library=<absolute path to library file>`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| CMake: `Could NOT find JNI` | Set `JAVA_HOME` to a JDK (not a JRE) before configuring; delete the build dir and reconfigure. |
| CMake: no `Visual Studio 17 2022` generator | Install VS 2022 Build Tools with the C++ workload. |
| Gradle: `frcsim native library directory not found` | Run the `cmake --install` step for your platform. |
| Java: `Failed to load the frcsim native library` | Library missing from `java.library.path`, or built for a different architecture than the JVM. |
| Java: `ABI version ... does not match` | Java and native built from different commits; rebuild both. |
