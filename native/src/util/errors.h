#pragma once

#include <stdexcept>

namespace frcsim {

// Exception types thrown by the core. The C ABI maps them to status codes:
//   std::invalid_argument  -> FRCSIM_ERR_INVALID_ARGUMENT
//   CapacityExceededError  -> FRCSIM_ERR_CAPACITY_EXCEEDED
//   NotFoundError          -> FRCSIM_ERR_NOT_FOUND
//   std::bad_alloc         -> FRCSIM_ERR_OUT_OF_MEMORY
//   anything else          -> FRCSIM_ERR_INTERNAL

/// A fixed capacity chosen at world creation (bodies, pieces, materials, types) is exhausted.
class CapacityExceededError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// A named or indexed entity does not exist.
class NotFoundError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

} // namespace frcsim
