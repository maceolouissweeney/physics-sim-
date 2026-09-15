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

JPH::RefConst<JPH::Shape> makeSphere(float radiusMeters) {
    if (!positiveFinite(radiusMeters)) {
        throw std::invalid_argument("sphere radius must be finite and > 0");
    }
    return new JPH::SphereShape(radiusMeters);
}

JPH::RefConst<JPH::Shape> makeBox(JPH::Vec3 halfExtentsMeters, float maxConvexRadiusMeters) {
    if (!positiveFinite(halfExtentsMeters.GetX()) || !positiveFinite(halfExtentsMeters.GetY()) ||
        !positiveFinite(halfExtentsMeters.GetZ())) {
        throw std::invalid_argument("box half extents must be finite and > 0");
    }
    const float convexRadiusMeters = std::min(maxConvexRadiusMeters, halfExtentsMeters.ReduceMin());
    return new JPH::BoxShape(halfExtentsMeters, convexRadiusMeters);
}

JPH::RefConst<JPH::Shape> makeZCylinder(float radiusMeters, float halfHeightMeters, float maxConvexRadiusMeters) {
    if (!positiveFinite(radiusMeters) || !positiveFinite(halfHeightMeters)) {
        throw std::invalid_argument("cylinder radius and half height must be finite and > 0");
    }
    const float convexRadiusMeters = std::min({maxConvexRadiusMeters, radiusMeters, halfHeightMeters});
    // Jolt cylinders run along Y; rotate +90 degrees about X so the axis is Z.
    JPH::RotatedTranslatedShapeSettings settings(
        JPH::Vec3::sZero(), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI),
        new JPH::CylinderShape(halfHeightMeters, radiusMeters, convexRadiusMeters));
    return unwrap(settings.Create(), "cylinder");
}

JPH::RefConst<JPH::Shape> makeConvexHull(std::span<const JPH::Vec3> pointsMeters, float maxConvexRadiusMeters) {
    if (pointsMeters.size() < 4) {
        throw std::invalid_argument("convex hull needs at least 4 points");
    }
    for (const JPH::Vec3& p : pointsMeters) {
        if (!std::isfinite(p.GetX()) || !std::isfinite(p.GetY()) || !std::isfinite(p.GetZ())) {
            throw std::invalid_argument("convex hull points must be finite");
        }
    }
    JPH::ConvexHullShapeSettings settings(pointsMeters.data(), static_cast<int>(pointsMeters.size()),
                                          maxConvexRadiusMeters);
    return unwrap(settings.Create(), "convex hull");
}

} // namespace frcsim
