#pragma once

namespace frcsim {

struct BatteryParams {
    float openCircuitVolts = 12.5f;        ///< CALIBRATE
    float internalResistanceOhms = 0.020f; ///< battery + main breaker + wiring. CALIBRATE
    float brownoutVolts = 6.75f;           ///< roboRIO 2 default brownout threshold
    float brownoutRecoveryVolts = 7.5f;    ///< outputs re-enable above this
};

/// Robot battery with internal resistance and brownout hysteresis (docs/models/swerve.md).
class Battery {
public:
    explicit Battery(const BatteryParams& params);

    /// Throws std::invalid_argument for invalid parameters.
    static void validate(const BatteryParams& params);

    /// Updates bus voltage from the total supply current drawn during the last substep.
    void update(float totalSupplyCurrentAmps) noexcept;

    [[nodiscard]] float voltageVolts() const { return m_voltageVolts; }
    [[nodiscard]] bool brownout() const { return m_brownout; }
    [[nodiscard]] float currentAmps() const { return m_currentAmps; }

private:
    BatteryParams m_params;
    float m_voltageVolts;
    float m_currentAmps = 0.0f;
    bool m_brownout = false;
};

} // namespace frcsim
