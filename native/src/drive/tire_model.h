#pragma once

#include <cmath>

namespace frcsim {

/// Tire–carpet friction (docs/models/swerve.md). All values CALIBRATE per tread with a pull test.
struct TireParams {
    float staticFriction = 1.1f;       ///< μ at zero slip
    float kineticFriction = 0.9f;      ///< μ while sliding
    float transitionSlipSpeed = 0.1f;  ///< m/s, width of the static→kinetic transition
};

/// Friction coefficient at a contact slip speed (m/s): Stribeck-style decay from static to kinetic.
[[nodiscard]] inline float tireFriction(const TireParams& tire, float slipSpeed) {
    const float x = slipSpeed / tire.transitionSlipSpeed;
    return tire.kineticFriction + (tire.staticFriction - tire.kineticFriction) * std::exp(-x * x);
}

} // namespace frcsim
