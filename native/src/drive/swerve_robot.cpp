#include "drive/swerve_robot.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

#include "util/errors.h"
#include "world/body_tag.h"
#include "world/layers.h"
#include "world/shapes.h"

namespace frcsim {
namespace {

constexpr double kTwoPi = 6.283185307179586;

float yawOf(JPH::Quat q) {
    return std::atan2(2.0f * (q.GetW() * q.GetZ() + q.GetX() * q.GetY()),
                      1.0f - 2.0f * (q.GetY() * q.GetY() + q.GetZ() * q.GetZ()));
}

JPH::RefConst<JPH::Shape> makeChassisShape(const SwerveDriveConfig& c) {
    const float halfHeight = 0.5f * c.bumperHeight;
    const JPH::Vec3 boxCenter(0.0f, 0.0f, c.bumperBottom + halfHeight);
    const auto box = makeBox(JPH::Vec3(c.frameHalfX, c.frameHalfY, halfHeight), kStaticConvexRadius);

    JPH::RotatedTranslatedShapeSettings raised(boxCenter, JPH::Quat::sIdentity(), box.GetPtr());
    JPH::Shape::ShapeResult raisedResult = raised.Create();
    if (raisedResult.HasError()) {
        throw std::invalid_argument(std::string("chassis shape: ") + raisedResult.GetError().c_str());
    }
    JPH::OffsetCenterOfMassShapeSettings offset(JPH::Vec3(c.comX, c.comY, c.comHeight) - boxCenter,
                                                raisedResult.Get().GetPtr());
    JPH::Shape::ShapeResult result = offset.Create();
    if (result.HasError()) {
        throw std::invalid_argument(std::string("chassis shape: ") + result.GetError().c_str());
    }
    return result.Get();
}

} // namespace

SwerveRobot::SwerveRobot(JPH::PhysicsSystem& physics, const MaterialTable& materials, const SwerveDriveConfig& config,
                         std::uint32_t index, float x, float y, float yaw)
    : m_physics(physics) {
    validate(config);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        throw std::invalid_argument("robot pose must be finite");
    }
    m_config = std::make_unique<SwerveDriveConfig>(config);
    m_state = std::make_unique<SwerveDrivetrainState>(*m_config);
    const SwerveDriveConfig& c = *m_config;
    const Material& bumper = materials.get(c.bumperMaterial);

    // Chassis body: bumper box above the carpet, explicit mass and box inertia.
    JPH::BodyCreationSettings settings(makeChassisShape(c).GetPtr(), JPH::RVec3(x, y, 0.0f),
                                       JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw), JPH::EMotionType::Dynamic,
                                       ObjectLayers::kRobot);
    const float lx = 2.0f * c.frameHalfX;
    const float ly = 2.0f * c.frameHalfY;
    const float lz = c.bumperHeight;
    const float ixx = c.mass * (ly * ly + lz * lz) / 12.0f;
    const float iyy = c.mass * (lx * lx + lz * lz) / 12.0f;
    const float izz = c.yawInertia > 0.0f ? c.yawInertia : c.mass * (lx * lx + ly * ly) / 12.0f;
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    settings.mMassPropertiesOverride.mMass = c.mass;
    settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(JPH::Vec3(ixx, iyy, izz));
    settings.mLinearDamping = 0.0f;
    settings.mAngularDamping = 0.0f;
    settings.mAllowSleeping = false;
    settings.mFriction = bumper.friction;
    settings.mRestitution = bumper.restitution;
    settings.mUserData = BodyTag{BodyKind::Robot, c.bumperMaterial, index}.encode();

    JPH::BodyInterface& bodies = physics.GetBodyInterfaceNoLock();
    JPH::Body* body = bodies.CreateBody(settings);
    if (body == nullptr) {
        throw CapacityExceededError("physics body capacity (max_bodies) exhausted while adding a robot");
    }
    m_body = body->GetID();
    bodies.AddBody(m_body, JPH::EActivation::Activate);

    // Vehicle constraint: one Jolt wheel per module, Z up, robot +X forward.
    JPH::VehicleConstraintSettings vehicle;
    vehicle.mUp = JPH::Vec3::sAxisZ();
    vehicle.mForward = JPH::Vec3::sAxisX();
    vehicle.mMaxPitchRollAngle = JPH::JPH_PI; // robots may tip
    for (const SwerveModuleConfig& m : c.modules) {
        JPH::Ref<JPH::WheelSettings> wheel = new JPH::WheelSettings();
        wheel->mPosition = JPH::Vec3(m.x, m.y, m.wheelRadius + c.suspension.travel);
        wheel->mSuspensionDirection = JPH::Vec3(0.0f, 0.0f, -1.0f);
        wheel->mSteeringAxis = JPH::Vec3::sAxisZ();
        wheel->mWheelUp = JPH::Vec3::sAxisZ();
        wheel->mWheelForward = JPH::Vec3::sAxisX();
        wheel->mSuspensionMinLength = 0.0f;
        wheel->mSuspensionMaxLength = c.suspension.travel;
        wheel->mSuspensionPreloadLength = 0.0f;
        wheel->mSuspensionSpring.mMode = JPH::ESpringMode::FrequencyAndDamping;
        wheel->mSuspensionSpring.mFrequency = c.suspension.frequency;
        wheel->mSuspensionSpring.mDamping = c.suspension.dampingRatio;
        wheel->mRadius = m.wheelRadius;
        wheel->mWidth = m.wheelWidth;
        vehicle.mWheels.push_back(wheel);
    }
    vehicle.mController = new SwerveVehicleControllerSettings(*m_config, *m_state, materials);

    m_constraint = new JPH::VehicleConstraint(*body, vehicle);
    m_tester = new JPH::VehicleCollisionTesterCastCylinder(ObjectLayers::kWheelProbe);
    m_constraint->SetVehicleCollisionTester(m_tester);
    physics.AddConstraint(m_constraint);
    physics.AddStepListener(m_constraint);

    m_lastYaw = yaw;
    m_continuousYaw = yaw;
    m_rng.seed(c.sensors.seed);
    m_gyroOrigin = yaw;
    m_gyroYaw = yaw;
}

SwerveRobot::~SwerveRobot() {
    m_physics.RemoveStepListener(m_constraint);
    m_physics.RemoveConstraint(m_constraint);
    m_constraint = nullptr;
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    bodies.RemoveBody(m_body);
    bodies.DestroyBody(m_body);
}

void SwerveRobot::setModuleVoltages(std::size_t index, float driveVolts, float steerVolts) {
    if (index >= m_state->modules.size()) {
        throw NotFoundError("swerve module index out of range");
    }
    if (!std::isfinite(driveVolts) || !std::isfinite(steerVolts)) {
        throw std::invalid_argument("module voltages must be finite");
    }
    m_state->modules[index].driveVoltageCommand = driveVolts;
    m_state->modules[index].steerVoltageCommand = steerVolts;
}

const SwerveModuleState& SwerveRobot::module(std::size_t index) const {
    if (index >= m_state->modules.size()) {
        throw NotFoundError("swerve module index out of range");
    }
    return m_state->modules[index];
}

RobotPose SwerveRobot::pose() const {
    const JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    RobotPose p;
    JPH::RVec3 position;
    bodies.GetPositionAndRotation(m_body, position, p.rotation);
    p.position = JPH::Vec3(position);
    p.linearVelocity = bodies.GetLinearVelocity(m_body);
    p.angularVelocity = bodies.GetAngularVelocity(m_body);
    return p;
}

void SwerveRobot::resetPose(float x, float y, float yaw) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        throw std::invalid_argument("robot pose must be finite");
    }
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    bodies.SetPositionAndRotation(m_body, JPH::RVec3(x, y, 0.0f), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw),
                                  JPH::EActivation::Activate);
    bodies.SetLinearAndAngularVelocity(m_body, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    for (JPH::Wheel* wheel : m_constraint->GetWheels()) {
        wheel->SetAngularVelocity(0.0f);
    }
    for (SwerveModuleState& module : m_state->modules) {
        module.wheelVelocity = 0.0f;
        module.steerVelocity = 0.0f;
    }
    m_lastYaw = yaw;
    m_continuousYaw = yaw;
    m_gyroOrigin = yaw;
    m_gyroElapsed = 0.0;
    m_gyroYaw = yaw;
}

void SwerveRobot::postStep(double dt) {
    const float yaw = yawOf(m_physics.GetBodyInterfaceNoLock().GetRotation(m_body));
    m_continuousYaw += std::remainder(static_cast<double>(yaw) - m_lastYaw, kTwoPi);
    m_lastYaw = yaw;

    const SensorParams& sensors = m_config->sensors;
    m_gyroElapsed += dt;
    double gyro = m_gyroOrigin + (m_continuousYaw - m_gyroOrigin) * (1.0 + sensors.gyroScaleError) +
                  sensors.gyroYawDriftRate * m_gyroElapsed;
    if (sensors.gyroYawNoise > 0.0f) {
        gyro += sensors.gyroYawNoise * m_normal(m_rng);
    }
    m_gyroYaw = gyro;
}

double SwerveRobot::measuredDriveRotorPosition(std::size_t index) const {
    const SwerveModuleState& state = module(index);
    const double position = state.wheelAngle * m_config->modules[index].driveGearRatio;
    const std::uint32_t counts = m_config->sensors.driveEncoderCountsPerRev;
    if (counts == 0) {
        return position;
    }
    const double step = kTwoPi / counts;
    return std::floor(position / step) * step;
}

Robots::Robots(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials) {}

std::uint32_t Robots::addSwerve(const SwerveDriveConfig& config, float x, float y, float yaw) {
    const auto index = static_cast<std::uint32_t>(m_swerve.size());
    m_swerve.push_back(std::make_unique<SwerveRobot>(m_physics, m_materials, config, index, x, y, yaw));
    return index;
}

SwerveRobot& Robots::swerve(std::uint32_t index) {
    if (index >= m_swerve.size()) {
        throw NotFoundError("robot index out of range");
    }
    return *m_swerve[index];
}

const SwerveRobot& Robots::swerve(std::uint32_t index) const {
    if (index >= m_swerve.size()) {
        throw NotFoundError("robot index out of range");
    }
    return *m_swerve[index];
}

void Robots::postStep(double dt) {
    for (auto& robot : m_swerve) {
        robot->postStep(dt);
    }
}

} // namespace frcsim
