// C ABI tests for materials, field, pieces, kinematic bodies, and shared-memory buffers.

#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <frcsim/frcsim_c.h>

namespace {

struct WorldDeleter {
    void operator()(frcsim_world* w) const { frcsim_world_destroy(w); }
};
using WorldPtr = std::unique_ptr<frcsim_world, WorldDeleter>;

WorldPtr makeWorld(uint32_t maxPieces = 1024) {
    frcsim_world_config config;
    frcsim_world_config_init(&config);
    config.max_pieces = maxPieces;
    frcsim_world* world = nullptr;
    EXPECT_EQ(frcsim_world_create(&config, &world), FRCSIM_OK) << frcsim_last_error();
    return WorldPtr(world);
}

std::string readRepoFile(const std::string& relative) {
    std::ifstream in(std::string(FRCSIM_REPO_DIR) + "/" + relative, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

TEST(CApiWorld, StatsPointerIsStableAndRefreshed) {
    WorldPtr world = makeWorld();
    const frcsim_world_stats* stats = frcsim_world_stats_ptr(world.get());
    ASSERT_NE(stats, nullptr);
    EXPECT_EQ(stats->substep_count, 0u);
    ASSERT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_OK);
    EXPECT_EQ(frcsim_world_stats_ptr(world.get()), stats);
    EXPECT_EQ(stats->substep_count, 5u);
    EXPECT_NEAR(stats->time_seconds, 0.02, 1e-12);
    EXPECT_GT(stats->last_step_wall_seconds, 0.0);
    EXPECT_EQ(frcsim_world_stats_ptr(nullptr), nullptr);
}

TEST(CApiWorld, MaterialsAddFindAndPair) {
    WorldPtr world = makeWorld();
    frcsim_material_id carpet = 255;
    ASSERT_EQ(frcsim_material_find(world.get(), "carpet", &carpet), FRCSIM_OK) << "standard materials exist";
    frcsim_material_id ice = 255;
    ASSERT_EQ(frcsim_material_add(world.get(), "ice", 0.05f, 0.1f, &ice), FRCSIM_OK);
    EXPECT_NE(ice, carpet);
    EXPECT_EQ(frcsim_material_set_pair(world.get(), ice, carpet, 0.2f, 0.0f), FRCSIM_OK);

    frcsim_material_id missing = 0;
    EXPECT_EQ(frcsim_material_find(world.get(), "unobtainium", &missing), FRCSIM_ERR_NOT_FOUND);
    EXPECT_EQ(frcsim_material_add(world.get(), "bad", -1.0f, 0.0f, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(frcsim_material_set_pair(world.get(), ice, 60, 0.2f, 0.0f), FRCSIM_ERR_NOT_FOUND);
}

TEST(CApiWorld, FieldJsonSpawnStepAndReadBuffers) {
    WorldPtr world = makeWorld();
    const std::string json = readRepoFile("fields/test-flat/field.json");
    ASSERT_EQ(frcsim_field_load_json(world.get(), json.data(), json.size()), FRCSIM_OK) << frcsim_last_error();

    frcsim_piece_type_id fuel = 0;
    ASSERT_EQ(frcsim_piece_type_find(world.get(), "fuel", &fuel), FRCSIM_OK);

    const float positions[] = {1, 1, 1, 2, 2, 1, 3, 3, 1};
    uint32_t indices[3] = {99, 99, 99};
    ASSERT_EQ(frcsim_pieces_spawn(world.get(), fuel, positions, nullptr, 3, indices), FRCSIM_OK)
        << frcsim_last_error();
    EXPECT_EQ(indices[2], 2u);
    EXPECT_EQ(frcsim_world_stats_ptr(world.get())->piece_high_water, 3u) << "stats refresh on spawn";

    frcsim_piece_buffers buffers{};
    ASSERT_EQ(frcsim_pieces_get_buffers(world.get(), &buffers), FRCSIM_OK);
    EXPECT_EQ(buffers.capacity, 1024u);
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_OK);
    }
    EXPECT_NEAR(buffers.positions_xyz[3 * 1 + 2], 0.075f, 0.005f);
    EXPECT_EQ(buffers.states[1], FRCSIM_PIECE_ON_FIELD);
    EXPECT_EQ(frcsim_world_stats_ptr(world.get())->pieces_simulated, 3u);

    ASSERT_EQ(frcsim_piece_set_state(world.get(), 1, FRCSIM_PIECE_IN_ROBOT), FRCSIM_OK);
    EXPECT_EQ(buffers.states[1], FRCSIM_PIECE_IN_ROBOT);
    const float dropAt[] = {5, 5, 0.5f};
    ASSERT_EQ(frcsim_piece_teleport(world.get(), 1, dropAt, nullptr, nullptr), FRCSIM_OK);
    EXPECT_EQ(buffers.states[1], FRCSIM_PIECE_ON_FIELD);
    ASSERT_EQ(frcsim_piece_despawn(world.get(), 2), FRCSIM_OK);
    EXPECT_EQ(buffers.states[2], FRCSIM_PIECE_INACTIVE);
    EXPECT_EQ(frcsim_world_stats_ptr(world.get())->pieces_simulated, 2u);
}

TEST(CApiWorld, PieceTypeDescriptorAndErrors) {
    WorldPtr world = makeWorld(4);
    frcsim_piece_type_desc desc;
    frcsim_piece_type_desc_init(&desc);
    desc.name = "cube";
    desc.shape = FRCSIM_PIECE_SHAPE_BOX;
    desc.half_extents[0] = desc.half_extents[1] = desc.half_extents[2] = 0.12f;
    desc.mass = 0.3f;
    frcsim_piece_type_id cube = 0;
    ASSERT_EQ(frcsim_piece_type_add(world.get(), &desc, &cube), FRCSIM_OK) << frcsim_last_error();
    EXPECT_EQ(frcsim_piece_type_add(world.get(), &desc, &cube), FRCSIM_ERR_INVALID_ARGUMENT) << "duplicate";

    desc.shape = 7;
    desc.name = "weird";
    EXPECT_EQ(frcsim_piece_type_add(world.get(), &desc, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);

    frcsim_piece_type_id missing = 0;
    EXPECT_EQ(frcsim_piece_type_find(world.get(), "note", &missing), FRCSIM_ERR_NOT_FOUND);

    const std::vector<float> five(15, 1.0f);
    EXPECT_EQ(frcsim_pieces_spawn(world.get(), cube, five.data(), nullptr, 5, nullptr), FRCSIM_ERR_CAPACITY_EXCEEDED);
    EXPECT_EQ(frcsim_world_stats_ptr(world.get())->piece_high_water, 0u);
    EXPECT_EQ(frcsim_pieces_spawn(world.get(), 42, five.data(), nullptr, 1, nullptr), FRCSIM_ERR_NOT_FOUND);
    EXPECT_EQ(frcsim_pieces_spawn(world.get(), cube, nullptr, nullptr, 1, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(frcsim_piece_despawn(world.get(), 0), FRCSIM_ERR_NOT_FOUND);
    EXPECT_EQ(frcsim_piece_set_state(world.get(), 0, 17), FRCSIM_ERR_INVALID_ARGUMENT);
}

TEST(CApiWorld, FieldPrimitivesAndKinematicPlow) {
    WorldPtr world = makeWorld();
    frcsim_material_id carpet = 0;
    ASSERT_EQ(frcsim_material_find(world.get(), "carpet", &carpet), FRCSIM_OK);
    ASSERT_EQ(frcsim_field_add_ground(world.get(), 0.0f, carpet, nullptr), FRCSIM_OK);
    const float wallCenter[] = {5, 0, 0.25f};
    const float wallHalf[] = {0.05f, 3, 0.25f};
    uint32_t wall = 99;
    ASSERT_EQ(frcsim_field_add_box(world.get(), "wall", wallCenter, wallHalf, nullptr, carpet, &wall), FRCSIM_OK);
    EXPECT_EQ(wall, 1u);
    const float bmin[] = {-10, -10, -1};
    const float bmax[] = {10, 10, 10};
    ASSERT_EQ(frcsim_field_set_bounds(world.get(), bmin, bmax), FRCSIM_OK);
    EXPECT_EQ(frcsim_field_set_bounds(world.get(), bmax, bmin), FRCSIM_ERR_INVALID_ARGUMENT);

    frcsim_piece_type_desc desc;
    frcsim_piece_type_desc_init(&desc);
    desc.name = "ball";
    desc.radius = 0.075f;
    desc.mass = 0.215f;
    frcsim_piece_type_id ball = 0;
    ASSERT_EQ(frcsim_piece_type_add(world.get(), &desc, &ball), FRCSIM_OK);
    const float at[] = {1.0f, 0, 0.075f};
    ASSERT_EQ(frcsim_pieces_spawn(world.get(), ball, at, nullptr, 1, nullptr), FRCSIM_OK);

    const float plowCenter[] = {0, 0, 0.1f};
    const float plowHalf[] = {0.45f, 0.45f, 0.08f};
    uint32_t plow = 99;
    ASSERT_EQ(frcsim_kinematic_add_box(world.get(), plowCenter, plowHalf, nullptr, carpet, &plow), FRCSIM_OK);
    for (int i = 1; i <= 50; ++i) {
        const float target[] = {0.03f * static_cast<float>(i), 0, 0.1f};
        ASSERT_EQ(frcsim_kinematic_move_to(world.get(), plow, target, nullptr, 0.02), FRCSIM_OK);
        ASSERT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_OK);
    }
    frcsim_piece_buffers buffers{};
    ASSERT_EQ(frcsim_pieces_get_buffers(world.get(), &buffers), FRCSIM_OK);
    EXPECT_GT(buffers.positions_xyz[0], 1.5f + 0.45f) << "plow should push the ball ahead of it";

    EXPECT_EQ(frcsim_kinematic_move_to(world.get(), 5, plowCenter, nullptr, 0.02), FRCSIM_ERR_NOT_FOUND);
}

TEST(CApiWorld, InvalidFieldJsonReportsMessage) {
    WorldPtr world = makeWorld();
    const std::string bad = R"({"schema": "frcsim.field/1", "statics": [{"type": "box"}]})";
    EXPECT_EQ(frcsim_field_load_json(world.get(), bad.data(), bad.size()), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(frcsim_last_error()).find("statics[0]"), std::string::npos) << frcsim_last_error();
    EXPECT_EQ(frcsim_field_load_json(world.get(), nullptr, 0), FRCSIM_ERR_INVALID_ARGUMENT);
}

} // namespace
