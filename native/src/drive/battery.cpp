#include "drive/battery.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace frcsim {

Battery::Battery(const BatteryParams& params) : m_params(params), m_voltageVolts(params.openCircuitVolts) {
    validate(params);
}

void Battery::validate(const BatteryParams& p) {
    if (!std::isfinite(p.openCircuitVolts) || p.openCircuitVolts <= 0.0f || !std::isfinite(p.internalResistanceOhms) ||
        p.internalResistanceOhms < 0.0f) {
        throw std::invalid_argument("battery voltage must be > 0 and resistance >= 0");
    }
    if (!std::isfinite(p.brownoutVolts) || !std::isfinite(p.brownoutRecoveryVolts) || p.brownoutVolts < 0.0f ||
        p.brownoutRecoveryVolts < p.brownoutVolts) {
        throw std::invalid_argument("battery brownout voltages must satisfy 0 <= brownout <= recovery");
    }
}

void Battery::update(float totalSupplyCurrentAmps) noexcept {
    m_currentAmps = std::isfinite(totalSupplyCurrentAmps) ? totalSupplyCurrentAmps : 0.0f;
    m_voltageVolts = std::max(0.0f, m_params.openCircuitVolts - m_params.internalResistanceOhms * m_currentAmps);
    if (!m_brownout && m_voltageVolts < m_params.brownoutVolts) {
        m_brownout = true;
    } else if (m_brownout && m_voltageVolts > m_params.brownoutRecoveryVolts) {
        m_brownout = false;
    }
}

} // namespace frcsim
