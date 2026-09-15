#pragma once

// Shared plumbing for the C ABI implementation files (capi_*.cpp). Not a public header.

#include <frcsim/frcsim_c.h>

#include <exception>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

#include <Jolt/Jolt.h>

#include "util/errors.h"
#include "world/world.h"

struct frcsim_world {
    explicit frcsim_world(const frcsim::WorldConfig& config);

    /// Copies module voltage inputs from shared memory into the robots. Throws on invalid (non-finite) input.
    void applyRobotInputs();
    /// Writes robot outputs into shared memory.
    void refreshRobotOutputs();
    void refreshStats() noexcept;

    frcsim::World world;
    frcsim_world_stats stats{};
    std::vector<std::unique_ptr<frcsim_swerve_robot_io>> robotIo; // stable addresses for shared memory
};

namespace frcsim::capi {

void setLastError(const char* message) noexcept;
void clearLastError() noexcept;

inline frcsim_status fail(frcsim_status status, const char* message) noexcept {
    setLastError(message);
    return status;
}

/// Runs body(), mapping exceptions to status codes. body must return frcsim_status.
template <typename Fn>
frcsim_status guarded(Fn&& body) noexcept {
    try {
        clearLastError();
        return body();
    } catch (const std::invalid_argument& e) {
        return fail(FRCSIM_ERR_INVALID_ARGUMENT, e.what());
    } catch (const CapacityExceededError& e) {
        return fail(FRCSIM_ERR_CAPACITY_EXCEEDED, e.what());
    } catch (const NotFoundError& e) {
        return fail(FRCSIM_ERR_NOT_FOUND, e.what());
    } catch (const std::bad_alloc&) {
        return fail(FRCSIM_ERR_OUT_OF_MEMORY, "out of memory");
    } catch (const std::exception& e) {
        return fail(FRCSIM_ERR_INTERNAL, e.what());
    } catch (...) {
        return fail(FRCSIM_ERR_INTERNAL, "unknown native exception");
    }
}

/// Like guarded(), but first validates the world handle.
template <typename Fn>
frcsim_status withWorld(frcsim_world* world, Fn&& body) noexcept {
    return guarded([&]() -> frcsim_status {
        if (world == nullptr) {
            return fail(FRCSIM_ERR_INVALID_ARGUMENT, "world must not be NULL");
        }
        return body(*world);
    });
}

inline void requireNonNull(const void* pointer, const char* name) {
    if (pointer == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be NULL");
    }
}

inline JPH::Vec3 toVec3(const float* xyz, const char* name) {
    requireNonNull(xyz, name);
    return JPH::Vec3(xyz[0], xyz[1], xyz[2]);
}

inline JPH::Vec3 toVec3OrZero(const float* xyz) {
    return xyz != nullptr ? JPH::Vec3(xyz[0], xyz[1], xyz[2]) : JPH::Vec3::sZero();
}

inline JPH::Quat toQuatOrIdentity(const float* xyzw) {
    return xyzw != nullptr ? JPH::Quat(xyzw[0], xyzw[1], xyzw[2], xyzw[3]) : JPH::Quat::sIdentity();
}

inline void writeIndex(uint32_t* out, std::uint32_t index) noexcept {
    if (out != nullptr) {
        *out = index;
    }
}

} // namespace frcsim::capi
