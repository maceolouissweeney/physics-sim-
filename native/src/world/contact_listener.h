#pragma once

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include "world/body_tag.h"
#include "world/materials.h"

namespace frcsim {

/**
 * Applies material-pair friction and restitution to every contact (decision D16).
 * Called from physics job threads; only reads the MaterialTable, which must not change during a step.
 */
class MaterialContactListener final : public JPH::ContactListener {
public:
    explicit MaterialContactListener(const MaterialTable& materials) : m_materials(materials) {}

    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold&,
                        JPH::ContactSettings& settings) override {
        apply(body1, body2, settings);
    }

    void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold&,
                            JPH::ContactSettings& settings) override {
        apply(body1, body2, settings);
    }

private:
    void apply(const JPH::Body& body1, const JPH::Body& body2, JPH::ContactSettings& settings) const noexcept {
        const Material combined = m_materials.combined(BodyTag::decodeMaterial(body1.GetUserData()),
                                                       BodyTag::decodeMaterial(body2.GetUserData()));
        settings.mCombinedFriction = combined.friction;
        settings.mCombinedRestitution = combined.restitution;
    }

    const MaterialTable& m_materials;
};

} // namespace frcsim
