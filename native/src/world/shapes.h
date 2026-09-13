#pragma once

#include <span>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace frcsim {

/// Convex radius for static field geometry: small so wall and rim edges stay sharp (decision D19).
inline constexpr float kStaticConvexRadius = 0.005f;

// Shape factories. All dimensions in meters; throw std::invalid_argument on invalid input.
// Cylinders use the Z axis (decision D18).

[[nodiscard]] JPH::RefConst<JPH::Shape> makeSphere(float radius);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeBox(JPH::Vec3 halfExtents, float maxConvexRadius);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeZCylinder(float radius, float halfHeight, float maxConvexRadius);
[[nodiscard]] JPH::RefConst<JPH::Shape> makeConvexHull(std::span<const JPH::Vec3> points, float maxConvexRadius);

} // namespace frcsim
