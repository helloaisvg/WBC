#pragma once

#include <string>

#include "rwb/types.hpp"

namespace rwb {

class SafetyShell {
public:
    explicit SafetyShell(ShellLimits limits);

    const ShellLimits& limits() const { return limits_; }
    bool frozen() const { return frozen_; }
    void freeze();

    ShellDecision project(const RobotState& state, const Action& requested) const;
    Action recovery_command(const RobotState& state) const;

    double arm_extension(const RobotState& state) const;
    double support_margin(const RobotState& state, const Action& projected) const;
    double stopping_distance(double speed) const;

private:
    ShellLimits limits_;
    bool frozen_ = false;

    double lateral_budget(const RobotState& state) const;
    double stopping_clearance(const RobotState& state) const;
};

std::string shell_reason_text(ShellReason reason);

}  // namespace rwb

