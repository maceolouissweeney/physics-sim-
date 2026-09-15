#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include <frcsim/frcsim_c.h>

namespace {

TEST(CApi, VersionAndAbi) {
    ASSERT_NE(frcsim_version_string(), nullptr);
    EXPECT_GT(std::strlen(frcsim_version_string()), 0u);
    EXPECT_EQ(frcsim_abi_version(), static_cast<uint32_t>(FRCSIM_ABI_VERSION));
}

TEST(CApi, StatusStringsAreNeverNull) {
    for (int s = 0; s <= 6; ++s) {
        EXPECT_NE(frcsim_status_string(static_cast<frcsim_status>(s)), nullptr);
    }
    EXPECT_STREQ(frcsim_status_string(FRCSIM_OK), "ok");
}

TEST(CApi, CreateStepDestroy) {
    frcsim_world* world = nullptr;
    ASSERT_EQ(frcsim_world_create(nullptr, &world), FRCSIM_OK) << frcsim_last_error();
    ASSERT_NE(world, nullptr);

    for (int i = 0; i < 50; ++i) {
        ASSERT_EQ(frcsim_world_step(world, 0.020, 5), FRCSIM_OK) << frcsim_last_error();
    }
    EXPECT_NEAR(frcsim_world_time_seconds(world), 1.0, 1e-12);
    frcsim_world_destroy(world);
}

TEST(CApi, CreateWithExplicitConfig) {
    frcsim_world_config config;
    frcsim_world_config_init(&config);
    EXPECT_EQ(config.struct_size, sizeof(frcsim_world_config));
    config.worker_threads = 2;
    config.gravity_z_meters_per_sec_sq = -1.0;

    frcsim_world* world = nullptr;
    ASSERT_EQ(frcsim_world_create(&config, &world), FRCSIM_OK) << frcsim_last_error();
    EXPECT_EQ(frcsim_world_step(world, 0.020, 5), FRCSIM_OK);
    frcsim_world_destroy(world);
}

TEST(CApi, NullOutputPointerIsInvalid) {
    EXPECT_EQ(frcsim_world_create(nullptr, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_STRNE(frcsim_last_error(), "");
}

TEST(CApi, StructSizeMismatchIsRejected) {
    frcsim_world_config config;
    frcsim_world_config_init(&config);
    config.struct_size = 4;
    frcsim_world* world = reinterpret_cast<frcsim_world*>(0x1);
    EXPECT_EQ(frcsim_world_create(&config, &world), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(world, nullptr) << "out pointer must be cleared on failure";
    EXPECT_NE(std::string(frcsim_last_error()).find("struct_size"), std::string::npos);
}

TEST(CApi, InvalidConfigValuesAreRejected) {
    frcsim_world_config config;
    frcsim_world_config_init(&config);
    config.max_bodies = 0;
    frcsim_world* world = nullptr;
    EXPECT_EQ(frcsim_world_create(&config, &world), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(frcsim_last_error()).find("max_bodies"), std::string::npos);
}

TEST(CApi, InvalidStepArgumentsAreRejected) {
    EXPECT_EQ(frcsim_world_step(nullptr, 0.02, 5), FRCSIM_ERR_INVALID_ARGUMENT);

    frcsim_world* world = nullptr;
    ASSERT_EQ(frcsim_world_create(nullptr, &world), FRCSIM_OK);
    EXPECT_EQ(frcsim_world_step(world, -0.02, 5), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(frcsim_last_error()).find("dt"), std::string::npos);
    EXPECT_EQ(frcsim_world_step(world, 0.02, 0), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_DOUBLE_EQ(frcsim_world_time_seconds(world), 0.0);
    frcsim_world_destroy(world);
}

TEST(CApi, SuccessClearsLastError) {
    frcsim_world_step(nullptr, 0.02, 5);
    ASSERT_STRNE(frcsim_last_error(), "");
    frcsim_world* world = nullptr;
    ASSERT_EQ(frcsim_world_create(nullptr, &world), FRCSIM_OK);
    EXPECT_STREQ(frcsim_last_error(), "");
    frcsim_world_destroy(world);
}

TEST(CApi, NullHandlesAreSafe) {
    frcsim_world_destroy(nullptr);
    EXPECT_DOUBLE_EQ(frcsim_world_time_seconds(nullptr), 0.0);
    frcsim_world_config_init(nullptr);
}

} // namespace
