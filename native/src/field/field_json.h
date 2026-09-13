#pragma once

#include <cstddef>
#include <string>
#include <string_view>

namespace frcsim {

class World;

struct FieldLoadSummary {
    std::string name;
    std::size_t staticsAdded = 0; ///< includes the ground slab
    std::size_t pieceTypesAdded = 0;
    std::size_t piecesSpawned = 0;
};

/**
 * Loads a `frcsim.field/1` document (docs/reference/field-json.md) into a world.
 * Throws std::invalid_argument (malformed document), NotFoundError, or CapacityExceededError.
 * Not atomic: on failure the world may be partially populated.
 */
FieldLoadSummary loadFieldJson(World& world, std::string_view jsonText);

} // namespace frcsim
