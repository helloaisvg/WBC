#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "rwb/monitor.hpp"

namespace rwb {

class NominalPlanner {
public:
    Action command(const RobotState& state, const Vec2& target,
                   double simulation_time) const;
    Action legacy_projection(const RobotState& state,
                             const Action& requested,
                             const ShellLimits& limits) const;
};

class RecoveryPolicy {
public:
    Action command(const RobotState& state, const SafetyShell& shell) const;
};

struct ExplorationProposal {
    Action action;
    ShellDecision shell;
    MonitorPrediction prediction;
    FeatureVector features{};
    bool probe = false;
    double score = 0.0;
    double scale = 1.0;
};

class BoundaryExplorer {
public:
    explicit BoundaryExplorer(ExplorationConfig config);

    ExplorationProposal propose(const RobotState& state,
                                const Action& nominal,
                                const SafetyShell& shell,
                                const RecoverabilityMonitor& monitor,
                                int step, Rng& rng) const;

    int period() const { return config_.probe_period_steps; }

private:
    ExplorationConfig config_;
};

class RecoverabilityGate {
public:
    GateDecision decide(const RobotState& state, const ShellDecision& shell,
                        const MonitorPrediction& prediction,
                        bool exploration_probe,
                        const SafetyShell& model_shell,
                        const RecoveryPolicy& recovery) const;
};

class BoundaryBook {
public:
    explicit BoundaryBook(MonitorLimits limits);

    bool observe(const FeatureVector& features,
                 const MonitorPrediction& prediction,
                 bool recovery_succeeded);

    int cell_count() const { return static_cast<int>(cells_.size()); }
    int positive_certificates() const { return positive_certificates_; }
    int negative_certificates() const { return negative_certificates_; }

private:
    struct Cell {
        std::array<std::uint8_t, kFeatureCount> key{};
        bool recovered = false;
    };

    MonitorLimits limits_;
    std::vector<Cell> cells_;
    int positive_certificates_ = 0;
    int negative_certificates_ = 0;
};

}  // namespace rwb

