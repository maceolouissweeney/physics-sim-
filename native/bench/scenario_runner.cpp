// frcsim_bench: runs fixed scenarios and reports wall-clock time per 20 ms robot period.
//
//   frcsim_bench [--scenario <name>|all] [--threads N] [--periods N] [--substeps N] [--json <path>]
//
// Scenarios map to the performance budgets in CLAUDE.md §7. Results are recorded in docs/perf/.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Jolt/Jolt.h>

#include "field/field_json.h"
#include "world/world.h"

namespace {

using namespace frcsim;

constexpr float kFuelRadiusMeters = 0.075f;
constexpr float kPeriod = 0.020f;

struct Options {
    std::string scenario = "all";
    int threads = 0;
    int periods = 500;
    int substeps = 5;
    int velocitySteps = WorldConfig{}.solverVelocitySteps;
    int positionSteps = WorldConfig{}.solverPositionSteps;
    std::string jsonPath;
};

struct Scenario {
    std::string name;
    std::string description;
    double budgetMs;
    int warmupPeriods;
    /// Populates the world and returns a per-period driver (called before each step with the period index).
    std::function<std::function<void(int)>(World&)> setup;
};

struct Result {
    std::string name;
    double budgetMs = 0.0;
    double minMs = 0.0, p50Ms = 0.0, p95Ms = 0.0, p99Ms = 0.0, maxMs = 0.0, meanMs = 0.0;
    double meanActiveBodies = 0.0;
    std::uint32_t pieces = 0;
    std::array<std::uint32_t, kPieceStateCount> stateCounts{};
    JPH::AABox outOfBoundsExits; ///< last recorded positions of pieces that left the bounds
};

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

void loadTestField(World& world) {
    loadFieldJson(world, readFile(std::string(FRCSIM_REPO_DIR) + "/fields/test-flat/field.json"));
}

/// Spawns nx*ny fuel in a grid resting on the carpet.
void spawnFuelGrid(World& world, int nx, int ny, float x0, float y0, float spacing) {
    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>(nx * ny * 3));
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            xyz.insert(xyz.end(), {x0 + spacing * static_cast<float>(i), y0 + spacing * static_cast<float>(j),
                                   kFuelRadiusMeters + 0.001f});
        }
    }
    world.pieces().spawn(*world.pieceTypes().find("fuel"), xyz, {}, {});
}

/// 60 kg, 0.9 m square robot stand-in with a traction-limited drive force (mu = 1: 590 N).
/// Bumper band 3-19 cm above the carpet.
std::uint32_t addRobotStandIn(World& world, float x, float y) {
    DrivenBodies::BoxDesc desc;
    desc.centerMeters = JPH::Vec3(x, y, 0.11f);
    desc.halfExtentsMeters = JPH::Vec3(0.45f, 0.45f, 0.08f);
    desc.massKg = 60.0f;
    desc.maxForceNewtons = 590.0f;
    desc.material = world.materials().require("bumper");
    return world.driven().addBox(desc);
}

/// A default 60 kg MK4i L2 swerve robot (docs/models/swerve.md) with bumper material.
std::uint32_t addSwerveRobot(World& world, float x, float y, float yaw) {
    SwerveDriveConfig config = makeRectangularSwerve(0.55f, 0.55f);
    config.bumperMaterial = world.materials().require("bumper");
    return world.robots().addSwerve(config, x, y, yaw);
}

/// Holds modules straight (a gentle P loop that is stable at the 50 Hz bench control rate) and drives
/// back and forth with a smooth voltage profile.
void driveSwerveBackAndForth(World& world, std::uint32_t index, float t, float phase) {
    constexpr float kSteerKp = 2.0f;
    SwerveRobot& robot = world.robots().swerve(index);
    const float driveVolts = 10.0f * std::sin(0.85f * t + phase);
    for (std::size_t m = 0; m < robot.moduleCount(); ++m) {
        const auto error = static_cast<float>(std::remainder(-robot.module(m).steerAngleRadians, 6.283185307179586));
        robot.setModuleVoltages(m, driveVolts, std::clamp(kSteerKp * error, -12.0f, 12.0f));
    }
}

/// Drives a stand-in along x(t) = center + amplitude * sin(omega * t) at fixed y.
void followSine(World& world, std::uint32_t robot, float t, float center, float amplitude, float omega, float y) {
    constexpr float kP = 3.0f; // position correction gain, 1/s
    const JPH::Vec3 p = world.driven().positionMeters(robot);
    const float xTarget = center + amplitude * std::sin(omega * t);
    const float vx = amplitude * omega * std::cos(omega * t) + kP * (xTarget - p.GetX());
    world.driven().setTargetVelocity(robot, vx, kP * (y - p.GetY()), 0.0f);
}

std::vector<Scenario> scenarios() {
    return {
        {"empty_field", "field + 1 robot stand-in, no pieces (fixed per-step overhead baseline)", 0.0, 25,
         [](World& world) {
             loadTestField(world);
             const std::uint32_t robot = addRobotStandIn(world, 2.0f, 1.0f);
             return std::function<void(int)>([&world, robot](int period) {
                 followSine(world, robot, static_cast<float>(period) * kPeriod, 2.0f, 1.0f, 0.8f, 1.0f);
             });
         }},
        {"sleeping_504", "504 fuel at rest (asleep) + 1 robot stand-in driving in open carpet", 0.5, 250,
         [](World& world) {
             loadTestField(world);
             spawnFuelGrid(world, 24, 21, 5.0f, 2.0f, 0.3f); // 504 pieces spread over the mid-field
             const std::uint32_t robot = addRobotStandIn(world, 2.0f, 1.0f);
             return std::function<void(int)>([&world, robot](int period) {
                 // Stays in x in [1, 3], away from the pieces.
                 followSine(world, robot, static_cast<float>(period) * kPeriod, 2.0f, 1.0f, 0.8f, 1.0f);
             });
         }},
        {"awake_360_plow", "360 packed fuel + 2 traction-limited 60 kg robot stand-ins driving through them", 2.0, 25,
         [](World& world) {
             loadTestField(world);
             spawnFuelGrid(world, 20, 18, 6.5f, 2.6f, 0.16f); // dense pile: 3.2 m x 2.9 m
             const std::uint32_t a = addRobotStandIn(world, 5.0f, 3.3f);
             const std::uint32_t b = addRobotStandIn(world, 11.0f, 4.6f);
             return std::function<void(int)>([&world, a, b](int period) {
                 const float t = static_cast<float>(period) * kPeriod;
                 followSine(world, a, t, 8.1f, 3.5f, 0.85f, 3.3f);
                 followSine(world, b, t, 8.1f, -3.5f, 0.85f, 4.6f);
             });
         }},
        {"swerve_2_360", "360 packed fuel + 2 full swerve robots (motors, tires, battery) driving through them",
         0.0, 25,
         [](World& world) {
             loadTestField(world);
             spawnFuelGrid(world, 20, 18, 6.5f, 2.6f, 0.16f);
             const std::uint32_t a = addSwerveRobot(world, 4.5f, 3.3f, 0.0f);
             const std::uint32_t b = addSwerveRobot(world, 11.7f, 4.6f, 3.14159265f);
             return std::function<void(int)>([&world, a, b](int period) {
                 const float t = static_cast<float>(period) * kPeriod;
                 driveSwerveBackAndForth(world, a, t, 0.0f);
                 driveSwerveBackAndForth(world, b, t, 0.0f);
             });
         }},
        {"swerve_6_504", "504 packed fuel + 6 swerve robots driving through them (stress)", 0.0, 25,
         [](World& world) {
             loadTestField(world);
             spawnFuelGrid(world, 24, 21, 6.2f, 2.4f, 0.16f);
             std::vector<std::uint32_t> robots;
             for (int i = 0; i < 3; ++i) {
                 const float y = 2.0f + 2.0f * static_cast<float>(i);
                 robots.push_back(addSwerveRobot(world, 4.0f, y, 0.0f));
                 robots.push_back(addSwerveRobot(world, 12.6f, y + 0.8f, 3.14159265f));
             }
             return std::function<void(int)>([&world, robots](int period) {
                 const float t = static_cast<float>(period) * kPeriod;
                 for (std::size_t i = 0; i < robots.size(); ++i) {
                     driveSwerveBackAndForth(world, robots[i], t, 0.3f * static_cast<float>(i));
                 }
             });
         }},
        {"full_504_plow", "504 packed fuel + 2 traction-limited robot stand-ins (stress, no budget)", 0.0, 25,
         [](World& world) {
             loadTestField(world);
             spawnFuelGrid(world, 24, 21, 6.2f, 2.4f, 0.16f);
             const std::uint32_t a = addRobotStandIn(world, 5.0f, 3.2f);
             const std::uint32_t b = addRobotStandIn(world, 11.5f, 4.8f);
             return std::function<void(int)>([&world, a, b](int period) {
                 const float t = static_cast<float>(period) * kPeriod;
                 followSine(world, a, t, 8.1f, 3.8f, 0.85f, 3.2f);
                 followSine(world, b, t, 8.1f, -3.8f, 0.85f, 4.8f);
             });
         }},
    };
}

double percentile(const std::vector<double>& sorted, double p) {
    const double rank = p * static_cast<double>(sorted.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(rank));
    const auto hi = static_cast<std::size_t>(std::ceil(rank));
    return sorted[lo] + (sorted[hi] - sorted[lo]) * (rank - static_cast<double>(lo));
}

Result run(const Scenario& scenario, const Options& options) {
    WorldConfig config;
    config.workerThreads = options.threads;
    config.solverVelocitySteps = static_cast<std::uint32_t>(options.velocitySteps);
    config.solverPositionSteps = static_cast<std::uint32_t>(options.positionSteps);
    World world(config);
    const auto drive = scenario.setup(world);

    int period = 0;
    for (; period < scenario.warmupPeriods; ++period) {
        drive(period);
        world.step(kPeriod, options.substeps);
    }

    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(options.periods));
    double activeSum = 0.0;
    for (int i = 0; i < options.periods; ++i, ++period) {
        drive(period);
        const auto start = std::chrono::steady_clock::now();
        world.step(kPeriod, options.substeps);
        const auto end = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::milli>(end - start).count());
        activeSum += world.stats().activeBodies;
    }

    std::vector<double> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    Result r;
    r.name = scenario.name;
    r.budgetMs = scenario.budgetMs;
    r.minMs = sorted.front();
    r.p50Ms = percentile(sorted, 0.50);
    r.p95Ms = percentile(sorted, 0.95);
    r.p99Ms = percentile(sorted, 0.99);
    r.maxMs = sorted.back();
    r.meanMs = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
    r.meanActiveBodies = activeSum / options.periods;
    r.pieces = world.stats().piecesSimulated;
    const PiecePool& pieces = world.pieces();
    for (std::size_t s = 0; s < kPieceStateCount; ++s) {
        r.stateCounts[s] = pieces.countInState(static_cast<PieceState>(s));
    }
    for (std::uint32_t i = 0; i < pieces.highWater(); ++i) {
        if (pieces.state(i) == PieceState::OutOfBounds) {
            r.outOfBoundsExits.Encapsulate(pieces.positionMeters(i));
        }
    }
    return r;
}

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--scenario") {
            o.scenario = next();
        } else if (arg == "--threads") {
            o.threads = std::stoi(next());
        } else if (arg == "--periods") {
            o.periods = std::max(1, std::stoi(next()));
        } else if (arg == "--substeps") {
            o.substeps = std::stoi(next());
        } else if (arg == "--velocity-steps") {
            o.velocitySteps = std::stoi(next());
        } else if (arg == "--position-steps") {
            o.positionSteps = std::stoi(next());
        } else if (arg == "--json") {
            o.jsonPath = next();
        } else if (arg == "--help" || arg == "-h") {
            std::printf("usage: frcsim_bench [--scenario <name>|all] [--threads N] [--periods N] [--substeps N] "
                        "[--json <path>]\nscenarios:\n");
            for (const Scenario& s : scenarios()) {
                std::printf("  %-16s %s (budget %.1f ms)\n", s.name.c_str(), s.description.c_str(), s.budgetMs);
            }
            std::exit(0);
        } else {
            std::fprintf(stderr, "unknown argument %s (try --help)\n", arg.c_str());
            std::exit(2);
        }
    }
    return o;
}

void writeJson(const std::string& path, const Options& options, const std::vector<Result>& results) {
    std::ofstream out(path);
    out << "{\n  \"threads\": " << options.threads << ",\n  \"substeps\": " << options.substeps
        << ",\n  \"periods\": " << options.periods << ",\n  \"hardwareThreads\": "
        << std::thread::hardware_concurrency() << ",\n  \"results\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const Result& r = results[i];
        out << "    {\"scenario\": \"" << r.name << "\", \"budgetMs\": " << r.budgetMs << ", \"p50Ms\": " << r.p50Ms
            << ", \"p95Ms\": " << r.p95Ms << ", \"p99Ms\": " << r.p99Ms << ", \"maxMs\": " << r.maxMs
            << ", \"meanMs\": " << r.meanMs << ", \"meanActiveBodies\": " << r.meanActiveBodies
            << ", \"pieces\": " << r.pieces << "}" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
}

} // namespace

int main(int argc, char** argv) {
    const Options options = parse(argc, argv);
    std::vector<Result> results;
    bool allWithinBudget = true;

    std::printf("frcsim_bench  threads=%d substeps=%d periods=%d velocity_steps=%d position_steps=%d\n\n",
                options.threads, options.substeps, options.periods, options.velocitySteps, options.positionSteps);
    std::printf("%-16s %7s %7s %7s %7s %7s %8s %7s %s\n", "scenario", "p50ms", "p95ms", "p99ms", "maxms", "budget",
                "active", "pieces", "");
    for (const Scenario& scenario : scenarios()) {
        if (options.scenario != "all" && options.scenario != scenario.name) {
            continue;
        }
        const Result r = run(scenario, options);
        const bool ok = r.budgetMs <= 0.0 || r.p50Ms <= r.budgetMs;
        allWithinBudget = allWithinBudget && ok;
        std::printf("%-16s %7.3f %7.3f %7.3f %7.3f %7.1f %8.1f %7u %s\n", r.name.c_str(), r.p50Ms, r.p95Ms, r.p99Ms,
                    r.maxMs, r.budgetMs, r.meanActiveBodies, r.pieces,
                    r.budgetMs <= 0.0 ? "" : (ok ? "OK" : "OVER BUDGET"));
        const auto oob = r.stateCounts[static_cast<std::size_t>(PieceState::OutOfBounds)];
        if (oob > 0) {
            const JPH::AABox& e = r.outOfBoundsExits;
            std::printf("%-16s   %u pieces out of bounds; exits x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]\n", "",
                        oob, e.mMin.GetX(), e.mMax.GetX(), e.mMin.GetY(), e.mMax.GetY(), e.mMin.GetZ(),
                        e.mMax.GetZ());
        }
        results.push_back(r);
    }
    if (results.empty()) {
        std::fprintf(stderr, "no scenario named '%s' (try --help)\n", options.scenario.c_str());
        return 2;
    }
    if (!options.jsonPath.empty()) {
        writeJson(options.jsonPath, options, results);
    }
    return allWithinBudget ? 0 : 1;
}
