// C ABI tests for swerve robots and their shared-memory I/O block.

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <frcsim/frcsim_c.h>

namespace {

struct WorldDeleter {
    void operator()(frcsim_world* w) const { frcsim_world_destroy(w); }
};
using WorldPtr = std::unique_ptr<frcsim_world, WorldDeleter>;

WorldPtr makeWorldWithCarpet() {
    frcsim_world* world = nullptr;
    EXPECT_EQ(frcsim_world_create(nullptr, &world), FRCSIM_OK) << frcsim_last_error();
    frcsim_material_id carpet = 0;
    EXPECT_EQ(frcsim_material_find(world, "carpet", &carpet), FRCSIM_OK);
    EXPECT_EQ(frcsim_field_add_ground(world, 0.0f, carpet, nullptr), FRCSIM_OK);
    return WorldPtr(world);
}

TEST(CApiRobot, MotorPresets) {
    frcsim_motor_params p{};
    ASSERT_EQ(frcsim_motor_params_preset(FRCSIM_MOTOR_KRAKEN_X60, 2, &p), FRCSIM_OK);
    EXPECT_FLOAT_EQ(p.stall_torque, 7.09f);
    EXPECT_EQ(p.count, 2);
    EXPECT_EQ(frcsim_motor_params_preset(99, 1, &p), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(frcsim_motor_params_preset(FRCSIM_MOTOR_NEO, 0, &p), FRCSIM_ERR_INVALID_ARGUMENT);
}

TEST(CApiRobot, AddDriveAndReadSharedIo) {
    WorldPtr world = makeWorldWithCarpet();
    frcsim_swerve_config config;
    frcsim_swerve_config_init(&config, 0.55f, 0.55f);
    EXPECT_EQ(config.module_count, 4u);
    EXPECT_FLOAT_EQ(config.modules[1].y, -0.275f) << "front-right module";

    uint32_t robot = 99;
    ASSERT_EQ(frcsim_robot_add_swerve(world.get(), &config, 2.0f, 3.0f, 0.5f, &robot), FRCSIM_OK)
        << frcsim_last_error();
    EXPECT_EQ(robot, 0u);
    frcsim_swerve_robot_io* io = frcsim_robot_io(world.get(), robot);
    ASSERT_NE(io, nullptr);
    EXPECT_EQ(io->module_count, 4u);
    EXPECT_NEAR(io->x, 2.0, 1e-5);
    EXPECT_NEAR(io->yaw, 0.5, 1e-5);
    EXPECT_DOUBLE_EQ(io->gyro_yaw, io->yaw) << "sensors are ideal by default";
    EXPECT_EQ(config.drive_encoder_counts_per_rev, 0u);
    EXPECT_EQ(config.sensor_seed, 1u);

    for (int i = 0; i < 50; ++i) { // settle 1 s
        ASSERT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_OK) << frcsim_last_error();
    }
    for (uint32_t m = 0; m < io->module_count; ++m) {
        io->modules[m].drive_voltage = 6.0f; // modules at angle 0 drive along robot +X (yaw 0.5 rad)
    }
    for (int i = 0; i < 50; ++i) {
        ASSERT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_OK);
    }
    EXPECT_EQ(frcsim_robot_io(world.get(), robot), io) << "I/O block address is stable";
    const double speed = std::hypot(io->vx, io->vy);
    EXPECT_GT(speed, 1.5);
    EXPECT_NEAR(std::atan2(io->vy, io->vx), 0.5, 0.05) << "drives along its heading";
    EXPECT_GT(io->modules[0].drive_rotor_position, 10.0);
    EXPECT_GT(io->modules[0].drive_rotor_velocity, 100.0f);
    EXPECT_NEAR(io->modules[0].drive_applied_voltage, 6.0f, 1e-3f);
    EXPECT_GT(io->modules[2].normal_force, 100.0f);
    EXPECT_LT(io->battery_voltage, 12.5f);
    EXPECT_EQ(io->brownout, 0u);

    ASSERT_EQ(frcsim_robot_reset_pose(world.get(), robot, 5.0f, 5.0f, 0.0f), FRCSIM_OK);
    EXPECT_NEAR(io->x, 5.0, 1e-5);
    EXPECT_NEAR(io->vx, 0.0f, 1e-5f);
}

TEST(CApiRobot, InvalidInputsAreRejected) {
    WorldPtr world = makeWorldWithCarpet();
    frcsim_swerve_config config;
    frcsim_swerve_config_init(&config, 0.55f, 0.55f);

    frcsim_swerve_config bad = config;
    bad.struct_size = 3;
    EXPECT_EQ(frcsim_robot_add_swerve(world.get(), &bad, 0, 0, 0, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    bad = config;
    bad.module_count = 9;
    EXPECT_EQ(frcsim_robot_add_swerve(world.get(), &bad, 0, 0, 0, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    bad = config;
    bad.modules[0].drive_neutral_mode = 5;
    EXPECT_EQ(frcsim_robot_add_swerve(world.get(), &bad, 0, 0, 0, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    bad = config;
    bad.mass = -1.0f;
    EXPECT_EQ(frcsim_robot_add_swerve(world.get(), &bad, 0, 0, 0, nullptr), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(frcsim_robot_io(world.get(), 0), nullptr) << "failed adds must not create I/O blocks";

    uint32_t robot = 0;
    ASSERT_EQ(frcsim_robot_add_swerve(world.get(), &config, 1, 1, 0, &robot), FRCSIM_OK);
    frcsim_robot_io(world.get(), robot)->modules[0].drive_voltage = NAN;
    EXPECT_EQ(frcsim_world_step(world.get(), 0.02, 5), FRCSIM_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(frcsim_robot_reset_pose(world.get(), 7, 0, 0, 0), FRCSIM_ERR_NOT_FOUND);
    EXPECT_EQ(frcsim_robot_io(nullptr, 0), nullptr);
}

} // namespace
