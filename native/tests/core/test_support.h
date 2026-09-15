#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Jolt/Jolt.h>

#include "world/world.h"

namespace frcsim::test {

inline constexpr float kFuelRadiusMeters = 0.075f; // REBUILT fuel: 0.150 m diameter
inline constexpr float kFuelMassKg = 0.215f;   // REBUILT fuel: 0.203-0.227 kg

inline PieceTypeId addFuelType(World& world, const char* name = "fuel") {
    PieceTypeDesc desc;
    desc.name = name;
    desc.shape = PieceShape::Sphere;
    desc.radiusMeters = kFuelRadiusMeters;
    desc.massKg = kFuelMassKg;
    desc.material = world.materials().require("foam");
    return world.pieceTypes().add(desc, world.materials());
}

inline void runPeriods(World& world, int periods, int substeps = 5) {
    for (int i = 0; i < periods; ++i) {
        world.step(0.020, substeps);
    }
}

/// Reads a file relative to the repository root.
inline std::string readRepoFile(const std::string& relativePath) {
    const std::string path = std::string(FRCSIM_REPO_DIR) + "/" + relativePath;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

} // namespace frcsim::test
