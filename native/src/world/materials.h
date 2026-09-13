#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "world/body_tag.h"

namespace frcsim {

struct Material {
    float friction = 0.5f;    ///< Coulomb friction coefficient, >= 0
    float restitution = 0.0f; ///< Coefficient of restitution, 0..1
};

/**
 * Named materials and their pairwise combined contact properties.
 *
 * The combined value for (a, b) is Jolt's default rule (friction = sqrt(fa*fb), restitution = max) unless
 * a pair override is set. Lookups are a single array read so the contact listener stays cheap.
 * Not thread-safe for writes; only mutate between world steps.
 */
class MaterialTable {
public:
    static constexpr std::size_t kMaxMaterials = 64; // power of two: ids are masked in combined()
    static constexpr MaterialId kDefault = 0;

    MaterialTable();

    /// Adds a material, or updates it if the name exists. Returns its id.
    MaterialId add(std::string_view name, const Material& material);

    [[nodiscard]] std::optional<MaterialId> find(std::string_view name) const;

    /// Like find(), but throws NotFoundError.
    [[nodiscard]] MaterialId require(std::string_view name) const;

    /// Overrides the combined properties for an unordered pair.
    void setPair(MaterialId a, MaterialId b, const Material& combined);

    [[nodiscard]] const Material& get(MaterialId id) const;
    [[nodiscard]] const std::string& name(MaterialId id) const;
    [[nodiscard]] std::size_t size() const { return m_materials.size(); }

    [[nodiscard]] Material combined(MaterialId a, MaterialId b) const noexcept {
        return m_pairs[pairIndex(a & (kMaxMaterials - 1), b & (kMaxMaterials - 1))];
    }

private:
    static constexpr std::size_t pairIndex(std::size_t a, std::size_t b) { return a * kMaxMaterials + b; }
    static void validate(const Material& material);
    void refreshCombinations(MaterialId id);

    std::vector<std::string> m_names;
    std::vector<Material> m_materials;
    std::array<Material, kMaxMaterials * kMaxMaterials> m_pairs{};
    std::array<bool, kMaxMaterials * kMaxMaterials> m_pairOverridden{};
};

/**
 * Registers starting-point FRC materials. All values are estimates to be replaced by measurements
 * (CLAUDE.md §6, Phase 6).
 */
void registerStandardFrcMaterials(MaterialTable& table);

} // namespace frcsim
