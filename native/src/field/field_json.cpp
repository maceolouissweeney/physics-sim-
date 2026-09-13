#include "field/field_json.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

using Json = nlohmann::json;

constexpr std::string_view kSchema = "frcsim.field/1";
constexpr float kDegreesToRadians = JPH::JPH_PI / 180.0f;

[[noreturn]] void fail(const std::string& where, const std::string& message) {
    throw std::invalid_argument("field json: " + where + ": " + message);
}

float toFloat(const Json& value, const std::string& where) {
    if (!value.is_number()) {
        fail(where, "expected a number");
    }
    const double v = value.get<double>();
    if (!std::isfinite(v)) {
        fail(where, "number must be finite");
    }
    return static_cast<float>(v);
}

float number(const Json& object, const char* key, const std::string& where) {
    const auto it = object.find(key);
    if (it == object.end()) {
        fail(where, std::string("missing '") + key + "'");
    }
    return toFloat(*it, where + "." + key);
}

float numberOr(const Json& object, const char* key, float fallback, const std::string& where) {
    return object.contains(key) ? number(object, key, where) : fallback;
}

JPH::Vec3 toVec3(const Json& value, const std::string& where) {
    if (!value.is_array() || value.size() != 3) {
        fail(where, "expected [x, y, z]");
    }
    return JPH::Vec3(toFloat(value[0], where), toFloat(value[1], where), toFloat(value[2], where));
}

JPH::Vec3 vec3(const Json& object, const char* key, const std::string& where) {
    const auto it = object.find(key);
    if (it == object.end()) {
        fail(where, std::string("missing '") + key + "'");
    }
    return toVec3(*it, where + "." + key);
}

std::string stringOr(const Json& object, const char* key, const std::string& fallback, const std::string& where) {
    const auto it = object.find(key);
    if (it == object.end()) {
        return fallback;
    }
    if (!it->is_string()) {
        fail(where + "." + key, "expected a string");
    }
    return it->get<std::string>();
}

const Json* optionalMember(const Json& object, const char* key, bool (Json::*check)() const noexcept,
                           const char* expected) {
    const auto it = object.find(key);
    if (it == object.end()) {
        return nullptr;
    }
    if (!((*it).*check)()) {
        fail(key, std::string("expected ") + expected);
    }
    return &*it;
}

JPH::Quat rotation(const Json& object, const std::string& where) {
    if (!object.contains("rpyDeg")) {
        return JPH::Quat::sIdentity();
    }
    // sEulerAngles applies X, then Y, then Z (RotZ * RotY * RotX): same as WPILib Rotation3d(roll, pitch, yaw).
    return JPH::Quat::sEulerAngles(vec3(object, "rpyDeg", where) * kDegreesToRadians);
}

MaterialId materialRef(World& world, const Json& object, const std::string& where) {
    const std::string name = stringOr(object, "material", "default", where);
    if (const auto id = world.materials().find(name)) {
        return *id;
    }
    fail(where, "unknown material '" + name + "'");
}

void loadMaterials(World& world, const Json& doc) {
    if (const Json* materials = optionalMember(doc, "materials", &Json::is_object, "an object")) {
        for (const auto& [name, m] : materials->items()) {
            const std::string where = "materials." + name;
            if (!m.is_object()) {
                fail(where, "expected an object");
            }
            world.materials().add(name, Material{number(m, "friction", where), number(m, "restitution", where)});
        }
    }
    if (const Json* pairs = optionalMember(doc, "materialPairs", &Json::is_array, "an array")) {
        for (std::size_t i = 0; i < pairs->size(); ++i) {
            const Json& p = (*pairs)[i];
            const std::string where = "materialPairs[" + std::to_string(i) + "]";
            if (!p.is_object()) {
                fail(where, "expected an object");
            }
            const std::string a = stringOr(p, "a", "", where);
            const std::string b = stringOr(p, "b", "", where);
            const auto idA = world.materials().find(a);
            const auto idB = world.materials().find(b);
            if (!idA || !idB) {
                fail(where, "unknown material '" + (idA ? b : a) + "'");
            }
            world.materials().setPair(*idA, *idB,
                                      Material{number(p, "friction", where), number(p, "restitution", where)});
        }
    }
}

std::size_t loadStatics(World& world, const Json& doc) {
    std::size_t added = 0;
    if (const Json* ground = optionalMember(doc, "ground", &Json::is_object, "an object")) {
        world.field().addGround(numberOr(*ground, "height", 0.0f, "ground"), materialRef(world, *ground, "ground"));
        ++added;
    }
    const Json* statics = optionalMember(doc, "statics", &Json::is_array, "an array");
    if (statics == nullptr) {
        return added;
    }
    for (std::size_t i = 0; i < statics->size(); ++i) {
        const Json& s = (*statics)[i];
        const std::string where = "statics[" + std::to_string(i) + "]";
        if (!s.is_object()) {
            fail(where, "expected an object");
        }
        const std::string type = stringOr(s, "type", "", where);
        const std::string name = stringOr(s, "name", "", where);
        const JPH::Vec3 center = vec3(s, "center", where);
        const JPH::Quat rot = rotation(s, where);
        const MaterialId material = materialRef(world, s, where);

        if (type == "box") {
            world.field().addBox(name, center, vec3(s, "halfExtents", where), rot, material);
        } else if (type == "cylinder") {
            world.field().addCylinder(name, center, number(s, "radius", where), number(s, "halfHeight", where), rot,
                                      material);
        } else if (type == "convexHull") {
            const auto it = s.find("points");
            if (it == s.end() || !it->is_array()) {
                fail(where, "convexHull needs 'points' array");
            }
            std::vector<JPH::Vec3> points;
            points.reserve(it->size());
            for (std::size_t k = 0; k < it->size(); ++k) {
                points.push_back(toVec3((*it)[k], where + ".points[" + std::to_string(k) + "]"));
            }
            world.field().addConvexHull(name, center, points, rot, material);
        } else {
            fail(where, "unknown static type '" + type + "' (expected box, cylinder, convexHull)");
        }
        ++added;
    }
    return added;
}

void loadBounds(World& world, const Json& doc) {
    if (const Json* bounds = optionalMember(doc, "bounds", &Json::is_object, "an object")) {
        world.field().setBounds(JPH::AABox(vec3(*bounds, "min", "bounds"), vec3(*bounds, "max", "bounds")));
    }
}

std::size_t loadPieceTypes(World& world, const Json& doc) {
    const Json* types = optionalMember(doc, "pieceTypes", &Json::is_object, "an object");
    if (types == nullptr) {
        return 0;
    }
    std::size_t added = 0;
    for (const auto& [name, t] : types->items()) {
        const std::string where = "pieceTypes." + name;
        if (!t.is_object()) {
            fail(where, "expected an object");
        }
        PieceTypeDesc desc;
        desc.name = name;
        const std::string shape = stringOr(t, "shape", "", where);
        if (shape == "sphere") {
            desc.shape = PieceShape::Sphere;
            desc.radius = number(t, "radius", where);
        } else if (shape == "cylinder") {
            desc.shape = PieceShape::Cylinder;
            desc.radius = number(t, "radius", where);
            desc.halfHeight = number(t, "halfHeight", where);
        } else if (shape == "box") {
            desc.shape = PieceShape::Box;
            const JPH::Vec3 h = vec3(t, "halfExtents", where);
            desc.halfExtents = {h.GetX(), h.GetY(), h.GetZ()};
        } else {
            fail(where, "unknown shape '" + shape + "' (expected sphere, cylinder, box)");
        }
        desc.mass = number(t, "mass", where);
        desc.material = materialRef(world, t, where);
        desc.maxAngularVelocity = numberOr(t, "maxAngularVelocity", desc.maxAngularVelocity, where);
        world.pieceTypes().add(desc, world.materials());
        ++added;
    }
    return added;
}

std::size_t loadPieceSpawns(World& world, const Json& doc) {
    const Json* spawns = optionalMember(doc, "pieceSpawns", &Json::is_array, "an array");
    if (spawns == nullptr) {
        return 0;
    }
    std::size_t spawned = 0;
    std::vector<float> xyz;
    for (std::size_t i = 0; i < spawns->size(); ++i) {
        const Json& s = (*spawns)[i];
        const std::string where = "pieceSpawns[" + std::to_string(i) + "]";
        if (!s.is_object()) {
            fail(where, "expected an object");
        }
        const std::string typeName = stringOr(s, "type", "", where);
        const auto type = world.pieceTypes().find(typeName);
        if (!type) {
            fail(where, "unknown piece type '" + typeName + "'");
        }
        xyz.clear();
        const bool hasPositions = s.contains("positions");
        const bool hasGrid = s.contains("grid");
        if (hasPositions == hasGrid) {
            fail(where, "needs exactly one of 'positions' or 'grid'");
        }
        if (hasPositions) {
            const Json& positions = s["positions"];
            if (!positions.is_array()) {
                fail(where + ".positions", "expected an array");
            }
            for (std::size_t k = 0; k < positions.size(); ++k) {
                const JPH::Vec3 p = toVec3(positions[k], where + ".positions[" + std::to_string(k) + "]");
                xyz.insert(xyz.end(), {p.GetX(), p.GetY(), p.GetZ()});
            }
        } else {
            const Json& grid = s["grid"];
            if (!grid.is_object()) {
                fail(where + ".grid", "expected an object");
            }
            const JPH::Vec3 origin = vec3(grid, "origin", where + ".grid");
            const JPH::Vec3 count = vec3(grid, "count", where + ".grid");
            const JPH::Vec3 spacing = vec3(grid, "spacing", where + ".grid");
            if (count.GetX() < 0 || count.GetY() < 0 || count.GetZ() < 0) {
                fail(where + ".grid", "count must be non-negative");
            }
            const auto nx = static_cast<std::size_t>(count.GetX());
            const auto ny = static_cast<std::size_t>(count.GetY());
            const auto nz = static_cast<std::size_t>(count.GetZ());
            const std::size_t total = nx * ny * nz;
            if (total > world.pieces().capacity()) {
                throw CapacityExceededError("field json: " + where + ": grid of " + std::to_string(total) +
                                            " pieces exceeds max_pieces");
            }
            xyz.reserve(total * 3);
            for (std::size_t k = 0; k < nz; ++k) {
                for (std::size_t j = 0; j < ny; ++j) {
                    for (std::size_t n = 0; n < nx; ++n) {
                        xyz.insert(xyz.end(), {origin.GetX() + static_cast<float>(n) * spacing.GetX(),
                                               origin.GetY() + static_cast<float>(j) * spacing.GetY(),
                                               origin.GetZ() + static_cast<float>(k) * spacing.GetZ()});
                    }
                }
            }
        }
        world.pieces().spawn(*type, xyz, {}, {});
        spawned += xyz.size() / 3;
    }
    return spawned;
}

} // namespace

FieldLoadSummary loadFieldJson(World& world, std::string_view jsonText) {
    try {
        const Json doc = Json::parse(jsonText.begin(), jsonText.end());
        if (!doc.is_object()) {
            fail("root", "expected an object");
        }
        const std::string schema = stringOr(doc, "schema", "", "root");
        if (schema != kSchema) {
            fail("schema", "expected '" + std::string(kSchema) + "', got '" + schema + "'");
        }

        FieldLoadSummary summary;
        summary.name = stringOr(doc, "name", "", "root");
        loadMaterials(world, doc);
        summary.staticsAdded = loadStatics(world, doc);
        loadBounds(world, doc);
        summary.pieceTypesAdded = loadPieceTypes(world, doc);
        summary.piecesSpawned = loadPieceSpawns(world, doc);
        world.optimizeBroadPhase();
        return summary;
    } catch (const Json::exception& e) {
        throw std::invalid_argument(std::string("field json: ") + e.what());
    }
}

} // namespace frcsim
