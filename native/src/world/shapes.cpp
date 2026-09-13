#include "world/shapes.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

namespace frcsim {
namespace {

bool positiveFinite(float value) {
    return std::isfinite(value) && value > 0.0f;
}

JPH::RefConst<JPH::Shape> unwrap(const JPH::Shape::ShapeResult& result, const char* what) {
    if (result.HasError()) {
        throw std::invalid_argument(std::string(what) + ": " + result.GetError().c_str());
    }
    return result.Get();
}

} // namespace

JPH::RefConst<JPH::Shape> makeSphere(float radius) {
    if (!positiveFinite(radius)) {
        throw std::invalid_argument("sphere radius must be finite and > 0");
    }
    return new JPH::SphereShape(radius);
}

JPH::RefConst<JPH::Shape> makeBox(JPH::Vec3 halfExtents, float maxConvexRadius) {
    if (!positiveFinite(halfExtents.GetX()) || !positiveFinite(halfExtents.GetY()) ||
        !positiveFinite(halfExtents.GetZ())) {
        throw std::invalid_argument("box half extents must be finite and > 0");
    }
    const float convexRadius = std::min(maxConvexRadius, halfExtents.ReduceMin());
    return new JPH::BoxShape(halfExtents, convexRadius);
}

JPH::RefConst<JPH::Shape> makeZCylinder(float radius, float halfHeight, float maxConvexRadius) {
    if (!positiveFinite(radius) || !positiveFinite(halfHeight)) {
        throw std::invalid_argument("cylinder radius and half height must be finite and > 0");
    }
    const float convexRadius = std::min({maxConvexRadius, radius, halfHeight});
    // Jolt cylinders run along Y; rotate +90 degrees about X so the axis is Z.
    JPH::RotatedTranslatedShapeSettings settings(JPH::Vec3::sZero(),
                                                 JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI),
                                                 new JPH::CylinderShape(halfHeight, radius, convexRadius));
    return unwrap(settings.Create(), "cylinder");
}

JPH::RefConst<JPH::Shape> makeConvexHull(std::span<const JPH::Vec3> points, float maxConvexRadius) {
    if (points.size() < 4) {
        throw std::invalid_argument("convex hull needs at least 4 points");
    }
    for (const JPH::Vec3& p : points) {
        if (!p.IsNaN() && std::isfinite(p.GetX()) && std::isfinite(p.GetY()) && std::isfinite(p.GetZ())) {
            continue;
        }
        throw std::invalid_argument("convex hull points must be finite");
    }
    JPH::ConvexHullShapeSettings settings(points.data(), static_cast<int>(points.size()), maxConvexRadius);
    return unwrap(settings.Create(), "convex hull");
}

} // namespace frcsim
