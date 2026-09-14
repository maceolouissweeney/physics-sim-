#include "drive/battery.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace frcsim {

Battery::Battery(const BatteryParams& params) : m_params(params), m_voltage(params.openCircuitVoltage) {
    validate(params);
}

void Battery::validate(const BatteryParams& p) {
    const auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
    if (!positive(p.openCircuitVoltage) || !std::isfinite(p.internalResistance) || p.internalResistance < 0.0f) {
        throw std::invalid_argument("battery voltage must be > 0 and resistance >= 0");
    }
    if (!std::isfinite(p.brownoutVoltage) || !std::isfinite(p.brownoutRecoveryVoltage) || p.brownoutVoltage < 0.0f ||
        p.brownoutRecoveryVoltage < p.brownoutVoltage) {
        throw std::invalid_argument("battery brownout voltages must satisfy 0 <= brownout <= recovery");
    }
}

void Battery::update(float totalSupplyCurrent) noexcept {
    m_current = std::isfinite(totalSupplyCurrent) ? totalSupplyCurrent : 0.0f;
    m_voltage = std::max(0.0f, m_params.openCircuitVoltage - m_params.internalResistance * m_current);
    if (!m_brownout && m_voltage < m_params.brownoutVoltage) {
        m_brownout = true;
    } else if (m_brownout && m_voltage > m_params.brownoutRecoveryVoltage) {
        m_brownout = false;
    }
}

} // namespace frcsim
