#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "core/test_support.h"
#include "field/field_json.h"
#include "util/errors.h"
#include "world/world.h"

namespace frcsim {
namespace {

TEST(FieldJson, LoadsRepositoryTestField) {
    World world(WorldConfig{});
    const FieldLoadSummary summary = loadFieldJson(world, test::readRepoFile("fields/test-flat/field.json"));

    EXPECT_EQ(summary.staticsAdded, 5u); // ground + 4 walls
    EXPECT_EQ(summary.pieceTypesAdded, 1u);
    EXPECT_EQ(summary.piecesSpawned, 0u);
    EXPECT_EQ(world.field().size(), 5u);
    EXPECT_EQ(world.field().primitiveName(1), "wall-blue");
    EXPECT_TRUE(world.pieceTypes().find("fuel"));
    EXPECT_FLOAT_EQ(world.field().bounds().mMax.GetX(), 17.54f);
}

TEST(FieldJson, InlineDocumentWithMaterialsTypesAndSpawns) {
    World world(WorldConfig{});
    const std::string doc = R"({
      "schema": "frcsim.field/1",
      "name": "inline",
      "materials": { "ice": { "friction": 0.05, "restitution": 0.1 } },
      "materialPairs": [ { "a": "ice", "b": "foam", "friction": 0.02, "restitution": 0.0 } ],
      "ground": { "material": "ice" },
      "statics": [
        { "type": "cylinder", "name": "post", "center": [3, 3, 0.5], "radius": 0.2, "halfHeight": 0.5,
          "rpyDeg": [0, 0, 45] },
        { "type": "convexHull", "center": [5, 5, 0],
          "points": [[0,0,0],[1,0,0],[0,1,0],[0,0,1]] }
      ],
      "pieceTypes": { "ball": { "shape": "sphere", "radius": 0.1, "mass": 0.3, "material": "foam" } },
      "pieceSpawns": [
        { "type": "ball", "positions": [[0, 0, 1], [1, 0, 1]] },
        { "type": "ball", "grid": { "origin": [0, 2, 0.1], "count": [3, 2, 1], "spacing": [0.25, 0.25, 0] } }
      ]
    })";
    const FieldLoadSummary summary = loadFieldJson(world, doc);

    EXPECT_EQ(summary.name, "inline");
    EXPECT_EQ(summary.staticsAdded, 3u);
    EXPECT_EQ(summary.piecesSpawned, 8u);
    EXPECT_EQ(world.pieces().countInState(PieceState::OnField), 8u);
    const MaterialId ice = world.materials().require("ice");
    const MaterialId foam = world.materials().require("foam");
    EXPECT_FLOAT_EQ(world.materials().combined(ice, foam).friction, 0.02f);
    EXPECT_NEAR(world.pieces().position(7).GetX(), 0.5f, 1e-5f); // grid (2, 1, 0)
    EXPECT_NEAR(world.pieces().position(7).GetY(), 2.25f, 1e-5f);
}

void expectInvalid(const std::string& doc, const std::string& messagePart) {
    World world(WorldConfig{});
    try {
        loadFieldJson(world, doc);
        FAIL() << "expected std::invalid_argument containing '" << messagePart << "'";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find(messagePart), std::string::npos) << e.what();
    }
}

TEST(FieldJson, ReportsErrorsWithLocation) {
    expectInvalid("{ not json", "field json");
    expectInvalid(R"({"schema": "frcsim.field/2"})", "schema");
    expectInvalid(R"({"schema": "frcsim.field/1", "ground": {"material": "nope"}})", "unknown material 'nope'");
    expectInvalid(R"({"schema": "frcsim.field/1", "statics": [{"type": "sphere", "center": [0,0,0]}]})",
                  "statics[0]");
    expectInvalid(R"({"schema": "frcsim.field/1", "statics": [{"type": "box", "center": [0,0]}]})",
                  "statics[0].center");
    expectInvalid(R"({"schema": "frcsim.field/1", "pieceSpawns": [{"type": "ghost", "positions": []}]})",
                  "unknown piece type 'ghost'");
    expectInvalid(R"({"schema": "frcsim.field/1", "materials": {"x": {"friction": "high", "restitution": 0}}})",
                  "materials.x.friction");
}

TEST(FieldJson, HugeGridIsRejectedBeforeAllocating) {
    World world(WorldConfig{});
    const std::string doc = R"({
      "schema": "frcsim.field/1",
      "pieceTypes": { "ball": { "shape": "sphere", "radius": 0.1, "mass": 0.3 } },
      "pieceSpawns": [ { "type": "ball", "grid": { "origin": [0,0,0], "count": [100000, 100000, 100000],
                                                    "spacing": [1,1,1] } } ]
    })";
    EXPECT_THROW(loadFieldJson(world, doc), CapacityExceededError);
}

} // namespace
} // namespace frcsim
