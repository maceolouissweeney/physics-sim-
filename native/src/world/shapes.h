#pragma once

#include <span>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace frcsim {

/// Convex radius for static field geometry: small so wall and rim edges stay sharp (decision D19).
inline constexpr float kStaticConvexRadiusMeters = 0.005f;

// Shape factories. Throw std::invalid_argument on invalid input. Cylinders use the Z axis (decision D18).

[[nodiscard]] JPH::RefConst<JPH::Shape> makeSphere(float radiusMeters);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeBox(JPH::Vec3 halfExtentsMeters, float maxConvexRadiusMeters);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeZCylinder(float radiusMeters, float halfHeightMeters,
                                                      float maxConvexRadiusMeters);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeConvexHull(std::span<const JPH::Vec3> pointsMeters,
                                                       float maxConvexRadiusMeters);

} // namespace frcsim
