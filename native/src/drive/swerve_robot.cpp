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

float yawRadiansOf(JPH::Quat q) {
    return std::atan2(2.0f * (q.GetW() * q.GetZ() + q.GetX() * q.GetY()),
                      1.0f - 2.0f * (q.GetY() * q.GetY() + q.GetZ() * q.GetZ()));
}

JPH::RefConst<JPH::Shape> makeChassisShape(const SwerveDriveConfig& c) {
    const float halfHeightMeters = 0.5f * c.bumperHeightMeters;
    const JPH::Vec3 boxCenterMeters(0.0f, 0.0f, c.bumperBottomMeters + halfHeightMeters);
    const auto box = makeBox(JPH::Vec3(c.frameHalfXMeters, c.frameHalfYMeters, halfHeightMeters), kStaticConvexRadiusMeters);

    JPH::RotatedTranslatedShapeSettings raised(boxCenterMeters, JPH::Quat::sIdentity(), box.GetPtr());
    JPH::Shape::ShapeResult raisedResult = raised.Create();
    if (raisedResult.HasError()) {
        throw std::invalid_argument(std::string("chassis shape: ") + raisedResult.GetError().c_str());
    }
    JPH::OffsetCenterOfMassShapeSettings offset(JPH::Vec3(c.comXMeters, c.comYMeters, c.comHeightMeters) - boxCenterMeters,
                                                raisedResult.Get().GetPtr());
    JPH::Shape::ShapeResult result = offset.Create();
    if (result.HasError()) {
        throw std::invalid_argument(std::string("chassis shape: ") + result.GetError().c_str());
    }
    return result.Get();
}

void requireFinitePose(float xMeters, float yMeters, float yawRadians) {
    if (!std::isfinite(xMeters) || !std::isfinite(yMeters) || !std::isfinite(yawRadians)) {
        throw std::invalid_argument("robot pose must be finite");
    }
}

} // namespace

SwerveRobot::SwerveRobot(JPH::PhysicsSystem& physics, const MaterialTable& materials, const SwerveDriveConfig& config,
                         std::uint32_t index, float xMeters, float yMeters, float yawRadians)
    : m_physics(physics) {
    validate(config);
    requireFinitePose(xMeters, yMeters, yawRadians);
    m_config = std::make_unique<SwerveDriveConfig>(config);
    m_state = std::make_unique<SwerveDrivetrainState>(*m_config);
    const SwerveDriveConfig& c = *m_config;
    const Material& bumper = materials.get(c.bumperMaterial);

    // Chassis body: bumper box above the carpet, explicit mass and box inertia.
    JPH::BodyCreationSettings settings(makeChassisShape(c).GetPtr(), JPH::RVec3(xMeters, yMeters, 0.0f),
                                       JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yawRadians), JPH::EMotionType::Dynamic,
                                       ObjectLayers::kRobot);
    const float lengthMeters = 2.0f * c.frameHalfXMeters;
    const float widthMeters = 2.0f * c.frameHalfYMeters;
    const float heightMeters = c.bumperHeightMeters;
    const float rollInertiaKgMetersSq = c.massKg * (widthMeters * widthMeters + heightMeters * heightMeters) / 12.0f;
    const float pitchInertiaKgMetersSq = c.massKg * (lengthMeters * lengthMeters + heightMeters * heightMeters) / 12.0f;
    const float yawInertiaKgMetersSq = c.yawInertiaKgMetersSq > 0.0f
                                           ? c.yawInertiaKgMetersSq
                                           : c.massKg * (lengthMeters * lengthMeters + widthMeters * widthMeters) / 12.0f;
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    settings.mMassPropertiesOverride.mMass = c.massKg;
    settings.mMassPropertiesOverride.mInertia =
        JPH::Mat44::sScale(JPH::Vec3(rollInertiaKgMetersSq, pitchInertiaKgMetersSq, yawInertiaKgMetersSq));
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
        wheel->mPosition = JPH::Vec3(m.xMeters, m.yMeters, m.wheelRadiusMeters + c.suspension.travelMeters);
        wheel->mSuspensionDirection = JPH::Vec3(0.0f, 0.0f, -1.0f);
        wheel->mSteeringAxis = JPH::Vec3::sAxisZ();
        wheel->mWheelUp = JPH::Vec3::sAxisZ();
        wheel->mWheelForward = JPH::Vec3::sAxisX();
        wheel->mSuspensionMinLength = 0.0f;
        wheel->mSuspensionMaxLength = c.suspension.travelMeters;
        wheel->mSuspensionPreloadLength = 0.0f;
        wheel->mSuspensionSpring.mMode = JPH::ESpringMode::FrequencyAndDamping;
        wheel->mSuspensionSpring.mFrequency = c.suspension.frequencyHz;
        wheel->mSuspensionSpring.mDamping = c.suspension.dampingRatio;
        wheel->mRadius = m.wheelRadiusMeters;
        wheel->mWidth = m.wheelWidthMeters;
        vehicle.mWheels.push_back(wheel);
    }
    vehicle.mController = new SwerveVehicleControllerSettings(*m_config, *m_state, materials);

    m_constraint = new JPH::VehicleConstraint(*body, vehicle);
    m_tester = new JPH::VehicleCollisionTesterCastCylinder(ObjectLayers::kWheelProbe);
    m_constraint->SetVehicleCollisionTester(m_tester);
    physics.AddConstraint(m_constraint);
    physics.AddStepListener(m_constraint);

    m_lastYawRadians = yawRadians;
    m_continuousYawRadians = yawRadians;
    m_rng.seed(c.sensors.seed);
    m_gyroOriginRadians = yawRadians;
    m_gyroYawRadians = yawRadians;
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
    m_state->modules[index].driveCommandVolts = driveVolts;
    m_state->modules[index].steerCommandVolts = steerVolts;
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
    JPH::RVec3 positionMeters;
    bodies.GetPositionAndRotation(m_body, positionMeters, p.rotation);
    p.positionMeters = JPH::Vec3(positionMeters);
    p.linearVelocityMetersPerSec = bodies.GetLinearVelocity(m_body);
    p.angularVelocityRadPerSec = bodies.GetAngularVelocity(m_body);
    return p;
}

void SwerveRobot::resetPose(float xMeters, float yMeters, float yawRadians) {
    requireFinitePose(xMeters, yMeters, yawRadians);
    JPH::BodyInterface& bodies = m_physics.GetBodyInterfaceNoLock();
    bodies.SetPositionAndRotation(m_body, JPH::RVec3(xMeters, yMeters, 0.0f),
                                  JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yawRadians), JPH::EActivation::Activate);
    bodies.SetLinearAndAngularVelocity(m_body, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    for (JPH::Wheel* wheel : m_constraint->GetWheels()) {
        wheel->SetAngularVelocity(0.0f);
    }
    for (SwerveModuleState& module : m_state->modules) {
        module.wheelVelocityRadPerSec = 0.0f;
        module.steerVelocityRadPerSec = 0.0f;
    }
    m_lastYawRadians = yawRadians;
    m_continuousYawRadians = yawRadians;
    m_gyroOriginRadians = yawRadians;
    m_gyroElapsedSeconds = 0.0;
    m_gyroYawRadians = yawRadians;
}

void SwerveRobot::postStep(double dtSeconds) {
    const float yawRadians = yawRadiansOf(m_physics.GetBodyInterfaceNoLock().GetRotation(m_body));
    m_continuousYawRadians += std::remainder(static_cast<double>(yawRadians) - m_lastYawRadians, kTwoPi);
    m_lastYawRadians = yawRadians;

    const SensorParams& sensors = m_config->sensors;
    m_gyroElapsedSeconds += dtSeconds;
    double gyroYawRadians = m_gyroOriginRadians +
                            (m_continuousYawRadians - m_gyroOriginRadians) * (1.0 + sensors.gyroScaleError) +
                            sensors.gyroYawDriftRateRadPerSec * m_gyroElapsedSeconds;
    if (sensors.gyroYawNoiseRadians > 0.0f) {
        gyroYawRadians += sensors.gyroYawNoiseRadians * m_standardNormal(m_rng);
    }
    m_gyroYawRadians = gyroYawRadians;
}

double SwerveRobot::measuredDriveRotorPositionRadians(std::size_t index) const {
    const SwerveModuleState& state = module(index);
    const SwerveModuleConfig& cfg = m_config->modules[index];
    const double positionRadians =
        state.wheelAngleRadians * cfg.driveGearRatio + state.steerAngleRadians * cfg.couplingGearRatio;
    const std::uint32_t counts = m_config->sensors.driveEncoderCountsPerRev;
    if (counts == 0) {
        return positionRadians;
    }
    const double countRadians = kTwoPi / counts;
    return std::floor(positionRadians / countRadians) * countRadians;
}

double SwerveRobot::driveRotorVelocityRadPerSec(std::size_t index) const {
    const SwerveModuleState& state = module(index);
    const SwerveModuleConfig& cfg = m_config->modules[index];
    return static_cast<double>(state.wheelVelocityRadPerSec) * cfg.driveGearRatio +
           static_cast<double>(state.steerVelocityRadPerSec) * cfg.couplingGearRatio;
}

Robots::Robots(JPH::PhysicsSystem& physics, const MaterialTable& materials)
    : m_physics(physics), m_materials(materials) {}

std::uint32_t Robots::addSwerve(const SwerveDriveConfig& config, float xMeters, float yMeters, float yawRadians) {
    const auto index = static_cast<std::uint32_t>(m_swerve.size());
    m_swerve.push_back(std::make_unique<SwerveRobot>(m_physics, m_materials, config, index, xMeters, yMeters, yawRadians));
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

void Robots::postStep(double dtSeconds) {
    for (auto& robot : m_swerve) {
        robot->postStep(dtSeconds);
    }
}

} // namespace frcsim
