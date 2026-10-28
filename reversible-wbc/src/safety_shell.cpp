#include "rwb/safety_shell.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace rwb {

SafetyShell::SafetyShell(ShellLimits limits) : limits_(limits) {}

void SafetyShell::freeze() { frozen_ = true; }

double SafetyShell::arm_extension(const RobotState& state) const {
    const double first = 0.31 * std::cos(state.joint_1);
    const double second = 0.28 * std::cos(state.joint_1 + state.joint_2);
    constexpr double minimum_reach = 0.12;
    constexpr double nominal_reach = 0.52;
    return clamp01((first + second - minimum_reach) /
                   (nominal_reach - minimum_reach));
}

double SafetyShell::stopping_distance(double speed) const {
    return 0.5 * speed * speed / limits_.stop_decel +
           std::abs(speed) * limits_.control_dt;
}

double SafetyShell::stopping_clearance(const RobotState& state) const {
    return std::max(0.0, state.obstacle_clearance - limits_.base_radius -
                             limits_.collision_margin);
}

double SafetyShell::lateral_budget(const RobotState& state) const {
    const double extension = arm_extension(state);
    const double mass_excess = std::max(0.0, state.payload_mass - 8.0);
    const double friction_factor =
        clamp(state.friction / 1.1, 0.72, 1.12);
    return limits_.max_lateral_accel * friction_factor *
           (1.0 - 0.20 * extension - 0.012 * mass_excess);
}

double SafetyShell::support_margin(const RobotState& state,
                                   const Action& projected) const {
    const double lateral =
        std::abs(state.linear_velocity * projected.angular_velocity);
    const double dynamic_shift =
        limits_.com_height * lateral / std::max(0.1, 9.81);
    const double body_shift = limits_.com_height * std::abs(state.roll);
    const double payload_shift =
        0.18 * std::tan(clamp(state.payload_angle, -0.9, 0.9));
    const double occupied = body_shift + payload_shift + dynamic_shift;
    return 1.0 - occupied / limits_.support_half_width;
}

ShellDecision SafetyShell::project(const RobotState& state,
                                   const Action& requested) const {
    if (!frozen_) {
        throw std::logic_error("safety shell must be frozen before control");
    }
    ShellDecision decision;
    decision.action = requested;
    const bool state_speed_outside_envelope =
        std::abs(state.linear_velocity) >
            limits_.max_linear_speed + 1e-9 ||
        std::abs(state.angular_velocity) >
            limits_.max_angular_speed + 1e-9;
    bool lateral_boundary_projected = false;

    const double raw_joint_1 = decision.action.joint_1_target;
    const double raw_joint_2 = decision.action.joint_2_target;
    decision.action.joint_1_target =
        clamp(raw_joint_1, limits_.min_joint_1 + limits_.joint_margin,
              limits_.max_joint_1 - limits_.joint_margin);
    decision.action.joint_2_target =
        clamp(raw_joint_2, limits_.min_joint_2 + limits_.joint_margin,
              limits_.max_joint_2 - limits_.joint_margin);

    if (std::abs(raw_joint_1 - decision.action.joint_1_target) > 1e-9 ||
        std::abs(raw_joint_2 - decision.action.joint_2_target) > 1e-9) {
        decision.reason |= ShellReason::JointRange;
    }

    const double raw_linear = decision.action.linear_velocity;
    const double raw_angular = decision.action.angular_velocity;
    decision.action.linear_velocity =
        clamp(raw_linear, -limits_.max_linear_speed, limits_.max_linear_speed);
    decision.action.angular_velocity =
        clamp(raw_angular, -limits_.max_angular_speed,
              limits_.max_angular_speed);

    if (std::abs(raw_linear - decision.action.linear_velocity) > 1e-9 ||
        std::abs(raw_angular - decision.action.angular_velocity) > 1e-9) {
        decision.reason |= ShellReason::Speed;
    }

    const double raw_balance = decision.action.balance_command;
    decision.action.balance_command =
        clamp(raw_balance, -limits_.max_balance_command,
              limits_.max_balance_command);
    if (std::abs(raw_balance - decision.action.balance_command) > 1e-9) {
        decision.reason |= ShellReason::Acceleration;
    }

    const double clearance = stopping_clearance(state);
    const double safe_speed =
        std::sqrt(std::max(0.0, 2.0 * limits_.stop_decel * clearance));
    if (std::abs(decision.action.linear_velocity) > safe_speed) {
        decision.action.linear_velocity =
            std::copysign(safe_speed, decision.action.linear_velocity);
        decision.reason |= ShellReason::StoppingDistance;
    }

    double desired_linear_accel =
        (decision.action.linear_velocity - state.linear_velocity) /
        limits_.linear_response_time;
    if (std::abs(desired_linear_accel) > limits_.max_linear_accel) {
        desired_linear_accel =
            std::copysign(limits_.max_linear_accel, desired_linear_accel);
        decision.action.linear_velocity =
            state.linear_velocity +
            desired_linear_accel * limits_.linear_response_time;
        decision.reason |= ShellReason::Acceleration;
    }
    decision.action.linear_velocity =
        clamp(decision.action.linear_velocity, -limits_.max_linear_speed,
              limits_.max_linear_speed);

    double desired_angular_accel =
        (decision.action.angular_velocity - state.angular_velocity) /
        limits_.angular_response_time;
    if (std::abs(desired_angular_accel) > limits_.max_angular_accel) {
        desired_angular_accel =
            std::copysign(limits_.max_angular_accel, desired_angular_accel);
        decision.action.angular_velocity =
            state.angular_velocity +
            desired_angular_accel * limits_.angular_response_time;
        decision.reason |= ShellReason::Acceleration;
    }
    decision.action.angular_velocity =
        clamp(decision.action.angular_velocity, -limits_.max_angular_speed,
              limits_.max_angular_speed);

    const double budget = lateral_budget(state);
    for (int iteration = 0; iteration < 8; ++iteration) {
        const double lateral = std::abs(decision.action.linear_velocity *
                                        decision.action.angular_velocity);
        if (lateral <= budget * 0.94) {
            break;
        }
        const double scale =
            std::sqrt(budget * 0.94 / std::max(lateral, 1e-9));
        decision.action.linear_velocity *= scale;
        decision.action.angular_velocity *= scale;
        decision.reason |= ShellReason::LateralStability;
        lateral_boundary_projected = true;
    }

    const double friction_limit =
        clamp(state.friction, 0.72, 1.35) * 9.81;
    const double accel = std::abs(
        (decision.action.linear_velocity - state.linear_velocity) /
        limits_.linear_response_time);
    const double lateral_accel =
        std::abs(decision.action.linear_velocity *
                 decision.action.angular_velocity);
    const double combined_accel =
        std::sqrt(accel * accel + lateral_accel * lateral_accel);
    if (combined_accel > friction_limit * 0.82) {
        const double scale =
            friction_limit * 0.82 / std::max(combined_accel, 1e-9);
        decision.action.linear_velocity =
            state.linear_velocity +
            (decision.action.linear_velocity - state.linear_velocity) * scale;
        decision.action.angular_velocity =
            state.angular_velocity +
            (decision.action.angular_velocity - state.angular_velocity) * scale;
        decision.reason |= ShellReason::Acceleration;
    }
    decision.action.linear_velocity =
        clamp(decision.action.linear_velocity, -limits_.max_linear_speed,
              limits_.max_linear_speed);
    decision.action.angular_velocity =
        clamp(decision.action.angular_velocity, -limits_.max_angular_speed,
              limits_.max_angular_speed);

    const double roll_utilization =
        std::abs(state.roll) / limits_.max_roll;
    decision.speed_utilization =
        std::abs(state.linear_velocity) / limits_.max_linear_speed;
    decision.acceleration_utilization =
        std::abs((decision.action.linear_velocity - state.linear_velocity) /
                 limits_.linear_response_time) /
        limits_.max_linear_accel;
    decision.lateral_utilization =
        std::abs(decision.action.linear_velocity *
                 decision.action.angular_velocity) /
        std::max(budget, 1e-9);
    decision.support_utilization =
        clamp01(1.0 - support_margin(state, decision.action));
    decision.joint_utilization = std::max(
        std::abs(state.joint_1) /
            std::max(std::abs(limits_.min_joint_1),
                     std::abs(limits_.max_joint_1)),
        std::abs(state.joint_2) / limits_.max_joint_2);
    decision.stopping_utilization =
        stopping_distance(state.linear_velocity) /
        std::max(clearance, 1e-6);

    if (decision.support_utilization >= 0.90 || roll_utilization >= 0.82) {
        decision.reason |= ShellReason::BodyLimit;
        decision.recovery_required = true;
    }

    if (state_speed_outside_envelope) {
        decision.reason |= ShellReason::Speed;
        decision.recovery_required = true;
    }

    if (stopping_distance(state.linear_velocity) > clearance) {
        decision.reason |= ShellReason::StoppingDistance;
        decision.recovery_required = true;
    }

    decision.critical_triggered =
        lateral_boundary_projected ||
        decision.support_utilization >= 0.90 ||
        roll_utilization >= 0.82 ||
        state_speed_outside_envelope ||
        stopping_distance(state.linear_velocity) > clearance;

    decision.triggered = decision.reason != ShellReason::None;
    if (decision.recovery_required) {
        decision.action = recovery_command(state);
    }
    return decision;
}

Action SafetyShell::recovery_command(const RobotState& state) const {
    if (!frozen_) {
        throw std::logic_error("safety shell must be frozen before control");
    }
    Action command;
    command.linear_velocity = 0.0;
    command.angular_velocity =
        clamp(-0.32 * state.angular_velocity, -0.22, 0.22);
    command.joint_1_target =
        clamp(0.0, limits_.min_joint_1, limits_.max_joint_1);
    command.joint_2_target =
        clamp(1.95, limits_.min_joint_2, limits_.max_joint_2);
    command.balance_command =
        clamp(-2.4 * state.roll - 0.35 * state.roll_rate -
                  0.29 * state.external_disturbance,
              -limits_.max_balance_command,
              limits_.max_balance_command);
    return command;
}

std::string shell_reason_text(ShellReason reason) {
    if (reason == ShellReason::None) {
        return "none";
    }
    std::ostringstream output;
    const std::array<std::pair<ShellReason, const char*>, 6> labels{{
        {ShellReason::Speed, "speed"},
        {ShellReason::Acceleration, "acceleration"},
        {ShellReason::LateralStability, "lateral"},
        {ShellReason::JointRange, "joint"},
        {ShellReason::StoppingDistance, "stopping"},
        {ShellReason::BodyLimit, "body"},
    }};
    bool first = true;
    for (const auto& [flag, label] : labels) {
        if (!has_reason(reason, flag)) {
            continue;
        }
        if (!first) {
            output << '|';
        }
        output << label;
        first = false;
    }
    return output.str();
}

}  // namespace rwb

