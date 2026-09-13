#pragma once

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace frcsim {

/**
 * Object layers decide which bodies can collide. Every body is assigned exactly one.
 *
 *   Static  - field carpet, walls, field elements. Never moves.
 *   Robot   - robot chassis, mechanism shapes, kinematic and driven bodies.
 *   Piece   - game pieces (on field or airborne).
 *   Sensor  - trigger volumes (goals, intake zones). Detect overlaps, generate no contact forces.
 */
namespace ObjectLayers {
inline constexpr JPH::ObjectLayer kStatic = 0;
inline constexpr JPH::ObjectLayer kRobot = 1;
inline constexpr JPH::ObjectLayer kPiece = 2;
inline constexpr JPH::ObjectLayer kSensor = 3;
inline constexpr JPH::uint kCount = 4;
} // namespace ObjectLayers

/**
 * Broadphase layers group object layers into separate acceleration trees.
 *
 * Pieces get their own tree, apart from robots: Jolt rebuilds a tree whenever anything in it moves, so
 * mixing a few always-moving robots with hundreds of mostly sleeping pieces would rebuild the big tree
 * every step (measured: 0.5 ms/period for 504 sleeping pieces vs 0.04 ms without them).
 */
namespace BroadPhaseLayers {
inline constexpr JPH::BroadPhaseLayer kNonMoving{0};
inline constexpr JPH::BroadPhaseLayer kRobot{1};
inline constexpr JPH::BroadPhaseLayer kPiece{2};
inline constexpr JPH::uint kCount = 3;
} // namespace BroadPhaseLayers

class BroadPhaseLayerMap final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::kCount; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        switch (layer) {
        case ObjectLayers::kStatic:
            return BroadPhaseLayers::kNonMoving;
        case ObjectLayers::kPiece:
            return BroadPhaseLayers::kPiece;
        default: // robots and sensors (sensors usually ride on robots)
            return BroadPhaseLayers::kRobot;
        }
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        if (layer == BroadPhaseLayers::kNonMoving) {
            return "NonMoving";
        }
        return layer == BroadPhaseLayers::kPiece ? "Piece" : "Robot";
    }
#endif
};

class ObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const override {
        switch (layer) {
        case ObjectLayers::kStatic:
            return false; // statics never query
        case ObjectLayers::kSensor:
            return broadPhaseLayer != BroadPhaseLayers::kNonMoving; // sensors only care about moving things
        default:
            return true;
        }
    }
};

class ObjectPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        const bool aStatic = a == ObjectLayers::kStatic;
        const bool bStatic = b == ObjectLayers::kStatic;
        const bool aSensor = a == ObjectLayers::kSensor;
        const bool bSensor = b == ObjectLayers::kSensor;
        if (aStatic && bStatic) {
            return false;
        }
        if (aSensor || bSensor) {
            // Sensors detect robots and pieces, not statics or other sensors.
            return !(aSensor && bSensor) && !aStatic && !bStatic;
        }
        return true;
    }
};

} // namespace frcsim
