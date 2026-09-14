#pragma once

namespace frcsim {

struct BatteryParams {
    float openCircuitVoltage = 12.5f;     ///< V. CALIBRATE
    float internalResistance = 0.020f;    ///< Ω, battery + main breaker + wiring. CALIBRATE
    float brownoutVoltage = 6.75f;        ///< V, roboRIO 2 default brownout threshold
    float brownoutRecoveryVoltage = 7.5f; ///< V, outputs re-enable above this
};

/// Robot battery with internal resistance and brownout hysteresis (docs/models/swerve.md).
class Battery {
public:
    explicit Battery(const BatteryParams& params);

    /// Throws std::invalid_argument for invalid parameters.
    static void validate(const BatteryParams& params);

    /// Updates bus voltage from the total supply current drawn during the last substep.
    void update(float totalSupplyCurrent) noexcept;

    [[nodiscard]] float voltage() const { return m_voltage; }
    [[nodiscard]] bool brownout() const { return m_brownout; }
    [[nodiscard]] float current() const { return m_current; }

private:
    BatteryParams m_params;
    float m_voltage;
    float m_current = 0.0f;
    bool m_brownout = false;
};

} // namespace frcsim
