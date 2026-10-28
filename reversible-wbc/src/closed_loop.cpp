#include "rwb/closed_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>

namespace rwb {

ClosedLoopRunner::ClosedLoopRunner(RuntimeConfig config,
                                   RecoverabilityMonitor& monitor)
    : config_(std::move(config)),
      shell_(config_.shell),
      planner_(),
      recovery_(),
      gate_(),
      explorer_(config_.exploration),
      monitor_(monitor),
      boundary_book_(config_.monitor) {
    shell_.freeze();
}

EpisodeResult ClosedLoopRunner::run(ControlMode mode, std::uint64_t seed,
                                    const std::string& telemetry_path) {
    WarehouseSim simulator(config_.scenario);
    simulator.reset(seed);
    Rng rng(seed * 6364136223846793005ULL + 1442695040888963407ULL);

    EpisodeResult result;
    result.seed = static_cast<int>(seed);
    result.payload_mass = simulator.state().payload_mass;
    result.friction = simulator.state().friction;

    std::ofstream telemetry;
    if (!telemetry_path.empty()) {
        telemetry.open(telemetry_path);
        telemetry << "step,time,x,y,yaw,v,w,roll,payload_angle,"
                     "rescue_probability,epistemic_uncertainty,"
                     "shell_triggered,gate_triggered,exploration_probe,"
                     "boundary_expanded,shell_reason,recovery_override,"
                     "clearance,stopping_util,support_util,lateral_util,"
                     "joint_1,joint_2,joint_velocity_1,joint_velocity_2,"
                     "payload_rate,payload_mass,friction,"
                     "external_disturbance,pre_x,pre_y,pre_yaw,pre_v,"
                     "pre_w,pre_roll,pre_payload_angle,requested_v,"
                     "requested_w,"
                     "requested_joint_1,requested_joint_2,"
                     "requested_balance,executed_v,executed_w,"
                     "executed_joint_1,executed_joint_2,executed_balance,"
                     "event\n";
    }

    std::vector<double> control_latencies_us;
    control_latencies_us.reserve(config_.scenario.max_steps);
    double probability_sum = 0.0;
    double uncertainty_sum = 0.0;
    int prediction_count = 0;

    for (int step = 0; step < config_.scenario.max_steps; ++step) {
        if (simulator.finished() || simulator.irreversible()) {
            break;
        }

        const auto control_begin = std::chrono::steady_clock::now();
        const RobotState state_before = simulator.state();
        const Action nominal =
            planner_.command(state_before, simulator.current_waypoint(),
                             simulator.time());

        Action executed = nominal;
        Action requested_action = nominal;
        MonitorPrediction prediction;
        prediction.rescue_probability = 1.0;
        prediction.epistemic_uncertainty = 0.0;
        bool shell_triggered = false;
        bool gate_triggered = false;
        bool exploration_probe = false;
        bool boundary_expanded = false;
        std::string applied_shell_reason = "none";
        bool recovery_override = false;
        double decision_clearance = 0.0;
        double decision_stopping_utilization = 0.0;
        double decision_support_utilization = 0.0;
        double decision_lateral_utilization = 0.0;

        if (mode == ControlMode::Legacy) {
            executed = planner_.legacy_projection(
                state_before, nominal, config_.shell);
            simulator.step(executed);
        } else {
            ExplorationProposal proposal = explorer_.propose(
                state_before, nominal, shell_, monitor_, step, rng);
            requested_action = proposal.action;
            exploration_probe = proposal.probe;
            GateDecision decision = gate_.decide(
                state_before, proposal.shell, proposal.prediction,
                proposal.probe, shell_, recovery_);
            executed = decision.action;
            prediction = decision.prediction;
            shell_triggered = proposal.shell.critical_triggered;
            gate_triggered = decision.replace_with_recovery ||
                             decision.governor_scaled;
            recovery_override = decision.replace_with_recovery;
            applied_shell_reason =
                shell_reason_text(proposal.shell.reason);
            decision_clearance = state_before.obstacle_clearance;
            decision_stopping_utilization =
                proposal.shell.stopping_utilization;
            decision_support_utilization =
                proposal.shell.support_utilization;
            decision_lateral_utilization =
                proposal.shell.lateral_utilization;

            if (exploration_probe) {
                const bool recovery_succeeded =
                    simulator.assess_recoverability(
                        state_before, proposal.shell.action, rng.raw());
                monitor_.adapt_online(proposal.features,
                                      recovery_succeeded);
                boundary_expanded = boundary_book_.observe(
                    proposal.features, proposal.prediction,
                    recovery_succeeded);
                ++result.exploratory_interactions;
                if (boundary_expanded) {
                    ++result.boundary_expansions;
                }
            }

            if (shell_triggered) {
                ++result.shell_triggers;
            }
            if (gate_triggered) {
                ++result.gate_triggers;
            }
            if (decision.replace_with_recovery) {
                ++result.recovery_actions;
            }
            probability_sum += prediction.rescue_probability;
            uncertainty_sum += prediction.epistemic_uncertainty;
            ++prediction_count;
            simulator.step(executed);
        }

        const auto control_end = std::chrono::steady_clock::now();
        control_latencies_us.push_back(
            std::chrono::duration<double, std::micro>(control_end -
                                                      control_begin)
                .count());

        const RobotState& state_after = simulator.state();
        if (telemetry) {
            telemetry << step << ',' << simulator.time() << ','
                      << state_after.position.x << ','
                      << state_after.position.y << ',' << state_after.yaw
                      << ',' << state_after.linear_velocity << ','
                      << state_after.angular_velocity << ','
                      << state_after.roll << ','
                      << state_after.payload_angle << ','
                      << prediction.rescue_probability << ','
                      << prediction.epistemic_uncertainty << ','
                      << (shell_triggered ? 1 : 0) << ','
                      << (gate_triggered ? 1 : 0) << ','
                      << (exploration_probe ? 1 : 0) << ','
                      << (boundary_expanded ? 1 : 0) << ','
                      << applied_shell_reason << ','
                      << (recovery_override ? 1 : 0) << ','
                      << decision_clearance << ','
                      << decision_stopping_utilization << ','
                      << decision_support_utilization << ','
                      << decision_lateral_utilization << ','
                      << state_before.joint_1 << ','
                      << state_before.joint_2 << ','
                      << state_before.joint_velocity_1 << ','
                      << state_before.joint_velocity_2 << ','
                      << state_before.payload_rate << ','
                      << state_before.payload_mass << ','
                      << state_before.friction << ','
                      << state_before.external_disturbance << ','
                      << state_before.position.x << ','
                      << state_before.position.y << ','
                      << state_before.yaw << ','
                      << state_before.linear_velocity << ','
                      << state_before.angular_velocity << ','
                      << state_before.roll << ','
                      << state_before.payload_angle << ','
                      << requested_action.linear_velocity << ','
                      << requested_action.angular_velocity << ','
                      << requested_action.joint_1_target << ','
                      << requested_action.joint_2_target << ','
                      << requested_action.balance_command << ','
                      << executed.linear_velocity << ','
                      << executed.angular_velocity << ','
                      << executed.joint_1_target << ','
                      << executed.joint_2_target << ','
                      << executed.balance_command << ','
                      << event_name(simulator.event()) << '\n';
        }
    }

    result.steps = static_cast<int>(control_latencies_us.size());
    result.elapsed_sim_time = simulator.time();
    result.reached_goal = simulator.finished();
    result.irreversible = simulator.irreversible();
    result.event = simulator.event();
    if (!config_.scenario.waypoints.empty()) {
        result.final_goal_distance =
            (config_.scenario.waypoints.back() - simulator.state().position)
                .norm();
    }
    if (prediction_count > 0) {
        result.mean_rescue_probability =
            probability_sum / static_cast<double>(prediction_count);
        result.mean_epistemic_uncertainty =
            uncertainty_sum / static_cast<double>(prediction_count);
    }
    if (!control_latencies_us.empty()) {
        result.mean_control_latency_us =
            std::accumulate(control_latencies_us.begin(),
                            control_latencies_us.end(), 0.0) /
            static_cast<double>(control_latencies_us.size());
        const std::size_t percentile_index =
            static_cast<std::size_t>(
                std::floor(0.95 *
                           static_cast<double>(
                               control_latencies_us.size() - 1)));
        std::nth_element(control_latencies_us.begin(),
                         control_latencies_us.begin() + percentile_index,
                         control_latencies_us.end());
        result.p95_control_latency_us =
            control_latencies_us[percentile_index];
    }
    return result;
}

}  // namespace rwb

