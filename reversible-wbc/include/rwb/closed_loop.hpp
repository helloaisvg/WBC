#pragma once

#include <string>

#include "rwb/dynamics.hpp"
#include "rwb/policy.hpp"

namespace rwb {

enum class ControlMode {
    Legacy,
    Reversible,
};

class ClosedLoopRunner {
public:
    ClosedLoopRunner(RuntimeConfig config, RecoverabilityMonitor& monitor);

    EpisodeResult run(ControlMode mode, std::uint64_t seed,
                      const std::string& telemetry_path = "");

    const BoundaryBook& boundary_book() const { return boundary_book_; }
    const SafetyShell& shell() const { return shell_; }

private:
    RuntimeConfig config_;
    SafetyShell shell_;
    NominalPlanner planner_;
    RecoveryPolicy recovery_;
    RecoverabilityGate gate_;
    BoundaryExplorer explorer_;
    RecoverabilityMonitor& monitor_;
    BoundaryBook boundary_book_;
};

}  // namespace rwb

