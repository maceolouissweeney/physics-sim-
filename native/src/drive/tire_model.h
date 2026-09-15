#pragma once

#include <cmath>

namespace frcsim {

/// Tire–carpet friction (docs/models/swerve.md). All values CALIBRATE per tread with a pull test.
struct TireParams {
    float staticFriction = 1.1f;                 ///< coefficient at zero slip
    float kineticFriction = 0.9f;                ///< coefficient while sliding
    float transitionSlipSpeedMetersPerSec = 0.1f; ///< width of the static→kinetic transition
};

/// Friction coefficient at a contact slip speed: Stribeck-style decay from static to kinetic.
[[nodiscard]] inline float tireFriction(const TireParams& tire, float slipSpeedMetersPerSec) {
    const float normalizedSlip = slipSpeedMetersPerSec / tire.transitionSlipSpeedMetersPerSec;
    return tire.kineticFriction +
           (tire.staticFriction - tire.kineticFriction) * std::exp(-normalizedSlip * normalizedSlip);
}

} // namespace frcsim
