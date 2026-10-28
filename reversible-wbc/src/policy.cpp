#include "rwb/policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rwb {

Action NominalPlanner::command(const RobotState& state, const Vec2& target,
                               double simulation_time) const {
    const Vec2 difference = target - state.position;
    const double distance = difference.norm();
    const double desired_yaw =
        distance > 1e-6 ? std::atan2(difference.y, difference.x) : state.yaw;
    const double heading_error = wrap_angle(desired_yaw - state.yaw);
    const double alignment =
        clamp(std::cos(heading_error) * 0.90 + 0.10, 0.12, 1.0);

    Action action;
    action.linear_velocity =
        clamp(0.13 + 0.42 * distance, 0.13, 0.88) * alignment;
    action.angular_velocity =
        clamp(1.85 * heading_error, -1.12, 1.12);
    action.joint_1_target =
        0.04 + 0.11 * std::sin(0.55 * simulation_time + 0.8);
    action.joint_2_target =
        1.08 + 0.13 * std::cos(0.47 * simulation_time);
    return action;
}

Action NominalPlanner::legacy_projection(const RobotState&,
                                         const Action& requested,
                                         const ShellLimits& limits) const {
    Action projected = requested;
    projected.linear_velocity =
        clamp(projected.linear_velocity, -limits.max_linear_speed,
              limits.max_linear_speed);
    projected.angular_velocity =
        clamp(projected.angular_velocity, -limits.max_angular_speed,
              limits.max_angular_speed);
    projected.joint_1_target =
        clamp(projected.joint_1_target, limits.min_joint_1,
              limits.max_joint_1);
    projected.joint_2_target =
        clamp(projected.joint_2_target, limits.min_joint_2,
              limits.max_joint_2);
    return projected;
}

Action RecoveryPolicy::command(const RobotState& state,
                               const SafetyShell& shell) const {
    return shell.recovery_command(state);
}

BoundaryExplorer::BoundaryExplorer(ExplorationConfig config)
    : config_(config) {}

ExplorationProposal BoundaryExplorer::propose(
    const RobotState& state, const Action& nominal, const SafetyShell& shell,
    const RecoverabilityMonitor& monitor, int step, Rng&) const {
    ExplorationProposal result;
    result.action = nominal;
    result.shell = shell.project(state, nominal);
    result.features =
        extract_features(state, result.shell.action, result.shell, shell);
    result.prediction = monitor.predict(result.features);

    if (config_.probe_period_steps <= 0 ||
        step % config_.probe_period_steps != 0) {
        return result;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    ExplorationProposal best = result;
    for (double scale : config_.candidate_scales) {
        Action candidate = nominal;
        candidate.linear_velocity *= scale;
        candidate.angular_velocity *=
            1.0 + 0.72 * (scale - 1.0);
        candidate.joint_1_target += 0.13 * (scale - 1.0);
        candidate.joint_2_target -= 0.07 * (scale - 1.0);

        ExplorationProposal proposal;
        proposal.action = candidate;
        proposal.shell = shell.project(state, candidate);
        proposal.features =
            extract_features(state, proposal.shell.action, proposal.shell,
                             shell);
        proposal.prediction = monitor.predict(proposal.features);
        if (proposal.prediction.rescue_probability <
            std::max(proposal.prediction.exploration_threshold,
                     shell.limits().exploration_probability_floor)) {
            continue;
        }
        const double probability_error =
            proposal.prediction.rescue_probability -
            config_.target_probability;
        const double boundary_score =
            std::exp(-sq(probability_error) / 0.012);
        proposal.score =
            config_.boundary_weight * boundary_score +
            config_.uncertainty_weight *
                proposal.prediction.epistemic_uncertainty +
            0.16 * (scale - 1.0);
        proposal.scale = scale;
        if (proposal.score > best_score) {
            best = proposal;
            best_score = proposal.score;
        }
    }

    if (best_score > 0.48) {
        best.probe = true;
        best.score = best_score;
        return best;
    }
    return result;
}

GateDecision RecoverabilityGate::decide(
    const RobotState& state, const ShellDecision& shell_decision,
    const MonitorPrediction& prediction, bool exploration_probe,
    const SafetyShell& model_shell,
    const RecoveryPolicy& recovery) const {
    GateDecision result;
    result.prediction = prediction;
    result.exploration_probe = exploration_probe;
    result.action = shell_decision.action;

    const bool probability_below_floor =
        prediction.rescue_probability <
        (exploration_probe
             ? std::max(prediction.exploration_threshold,
                        model_shell.limits().exploration_probability_floor)
             : std::max(prediction.strict_threshold,
                        model_shell.limits().strict_recoverability_floor));
    const double uncertainty_ceiling =
        exploration_probe
            ? model_shell.limits().epistemic_ceiling * 1.35
            : model_shell.limits().epistemic_ceiling;
    const bool uncertainty_too_high =
        prediction.epistemic_uncertainty > uncertainty_ceiling;

    const double hard_stop_threshold =
        std::max(0.32,
                 prediction.exploration_threshold * 0.66);
    const bool hard_stop =
        prediction.rescue_probability < hard_stop_threshold;
    if (shell_decision.recovery_required || hard_stop ||
        uncertainty_too_high) {
        result.action =
            model_shell.project(state, recovery.command(state, model_shell))
                .action;
        result.replace_with_recovery = true;
    } else if (probability_below_floor) {
        const double threshold =
            exploration_probe
                ? std::max(prediction.exploration_threshold,
                           model_shell.limits()
                               .exploration_probability_floor)
                : std::max(prediction.strict_threshold,
                           model_shell.limits()
                               .strict_recoverability_floor);
        const double normalized_deficit =
            (threshold - prediction.rescue_probability) /
            std::max(threshold - hard_stop_threshold, 1e-6);
        const double command_scale =
            clamp(1.0 - 0.72 * normalized_deficit, 0.20, 0.82);
        Action governed = shell_decision.action;
        governed.linear_velocity *= command_scale;
        governed.angular_velocity *= command_scale;
        result.action = model_shell.project(state, governed).action;
        result.governor_scaled = true;
    } else {
        result.action = shell_decision.action;
    }
    return result;
}

BoundaryBook::BoundaryBook(MonitorLimits limits)
    : limits_(limits) {}

bool BoundaryBook::observe(const FeatureVector& features,
                           const MonitorPrediction& prediction,
                           bool recovery_succeeded) {
    const bool on_boundary =
        prediction.rescue_probability >= limits_.boundary_low &&
        prediction.rescue_probability <= limits_.boundary_high;
    const bool uncertain =
        prediction.epistemic_uncertainty > 0.13;
    if (!on_boundary && !uncertain) {
        return false;
    }

    Cell candidate;
    for (std::size_t index = 0; index < kFeatureCount; ++index) {
        candidate.key[index] = static_cast<std::uint8_t>(
            std::min(15, static_cast<int>(
                             std::floor(clamp01(features[index]) * 16.0))));
    }
    candidate.recovered = recovery_succeeded;
    for (const Cell& cell : cells_) {
        if (cell.key == candidate.key) {
            return false;
        }
    }
    cells_.push_back(candidate);
    if (recovery_succeeded) {
        ++positive_certificates_;
    } else {
        ++negative_certificates_;
    }
    return true;
}

}  // namespace rwb

