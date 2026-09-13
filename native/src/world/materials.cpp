#include "world/materials.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "util/errors.h"

namespace frcsim {

MaterialTable::MaterialTable() {
    add("default", Material{0.5f, 0.0f});
}

void MaterialTable::validate(const Material& material) {
    if (!std::isfinite(material.friction) || material.friction < 0.0f) {
        throw std::invalid_argument("material friction must be finite and >= 0");
    }
    if (!std::isfinite(material.restitution) || material.restitution < 0.0f || material.restitution > 1.0f) {
        throw std::invalid_argument("material restitution must be in [0, 1]");
    }
}

MaterialId MaterialTable::add(std::string_view name, const Material& material) {
    if (name.empty()) {
        throw std::invalid_argument("material name must not be empty");
    }
    validate(material);

    if (const auto existing = find(name)) {
        m_materials[*existing] = material;
        refreshCombinations(*existing);
        return *existing;
    }
    if (m_materials.size() >= kMaxMaterials) {
        throw CapacityExceededError("too many materials (max " + std::to_string(kMaxMaterials) + ")");
    }
    const auto id = static_cast<MaterialId>(m_materials.size());
    m_names.emplace_back(name);
    m_materials.push_back(material);
    refreshCombinations(id);
    return id;
}

std::optional<MaterialId> MaterialTable::find(std::string_view name) const {
    const auto it = std::find(m_names.begin(), m_names.end(), name);
    if (it == m_names.end()) {
        return std::nullopt;
    }
    return static_cast<MaterialId>(it - m_names.begin());
}

MaterialId MaterialTable::require(std::string_view name) const {
    if (const auto id = find(name)) {
        return *id;
    }
    throw NotFoundError("unknown material '" + std::string(name) + "'");
}

void MaterialTable::setPair(MaterialId a, MaterialId b, const Material& combined) {
    if (a >= m_materials.size() || b >= m_materials.size()) {
        throw NotFoundError("material pair references an unknown material id");
    }
    validate(combined);
    m_pairs[pairIndex(a, b)] = combined;
    m_pairs[pairIndex(b, a)] = combined;
    m_pairOverridden[pairIndex(a, b)] = true;
    m_pairOverridden[pairIndex(b, a)] = true;
}

const Material& MaterialTable::get(MaterialId id) const {
    if (id >= m_materials.size()) {
        throw NotFoundError("unknown material id " + std::to_string(id));
    }
    return m_materials[id];
}

const std::string& MaterialTable::name(MaterialId id) const {
    if (id >= m_names.size()) {
        throw NotFoundError("unknown material id " + std::to_string(id));
    }
    return m_names[id];
}

void MaterialTable::refreshCombinations(MaterialId id) {
    const Material& self = m_materials[id];
    for (std::size_t other = 0; other < m_materials.size(); ++other) {
        if (m_pairOverridden[pairIndex(id, other)]) {
            continue;
        }
        const Material& o = m_materials[other];
        const Material combined{std::sqrt(self.friction * o.friction), std::max(self.restitution, o.restitution)};
        m_pairs[pairIndex(id, other)] = combined;
        m_pairs[pairIndex(other, id)] = combined;
    }
}

void registerStandardFrcMaterials(MaterialTable& table) {
    // CALIBRATE: every value below is an estimate (see CLAUDE.md §6.2 and Phase 6).
    const MaterialId carpet = table.add("carpet", Material{1.0f, 0.1f});
    table.add("polycarbonate", Material{0.4f, 0.5f});
    table.add("aluminum", Material{0.35f, 0.3f});
    table.add("bumper", Material{0.65f, 0.08f}); // maple-sim's tuned bumper values
    const MaterialId foam = table.add("foam", Material{0.7f, 0.5f});
    // Foam fuel on carpet: the carpet absorbs energy, and friction is high.
    table.setPair(foam, carpet, Material{0.8f, 0.35f});
}

} // namespace frcsim
