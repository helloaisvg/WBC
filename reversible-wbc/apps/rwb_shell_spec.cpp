#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "rwb/safety_shell.hpp"

namespace {

std::string parse_string(int argc, char** argv, const std::string& name,
                         const std::string& fallback) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return argv[index + 1];
        }
    }
    return fallback;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string output_path =
        parse_string(argc, argv, "--out", "outputs/shell_spec.txt");
    rwb::ShellLimits limits;
    rwb::SafetyShell shell(limits);
    shell.freeze();

    constexpr double sample_speeds[] = {-1.05, -0.78, 0.0, 0.43, 1.12};
    constexpr double sample_angular[] = {-1.45, -0.92, 0.0, 0.68, 1.36};
    constexpr double sample_masses[] = {3.0, 8.0, 14.0};
    constexpr double sample_friction[] = {0.78, 1.08, 1.38};
    constexpr double sample_roll[] = {-0.28, 0.0, 0.28};
    constexpr double sample_payload_angle[] = {-0.62, 0.0, 0.62};
    constexpr double sample_clearance[] = {0.09, 0.45, 1.40};
    constexpr double requested_linear[] = {-1.32, 1.32};
    constexpr double requested_angular[] = {-1.72, 1.72};
    constexpr double requested_joint_1[] = {-1.35, 1.18};
    constexpr double requested_joint_2[] = {-0.12, 2.55};

    long long cases = 0;
    long long counterexamples = 0;
    long long recovery_commands = 0;
    double maximum_speed_utilization = 0.0;
    double maximum_acceleration_utilization = 0.0;
    double maximum_lateral_utilization = 0.0;
    double maximum_support_utilization = 0.0;
    std::string first_counterexample;

    for (double speed : sample_speeds) {
        for (double angular : sample_angular) {
            for (double mass : sample_masses) {
                for (double friction : sample_friction) {
                    for (double roll : sample_roll) {
                        for (double payload_angle :
                             sample_payload_angle) {
                            for (double clearance : sample_clearance) {
                                rwb::RobotState state;
                                state.linear_velocity = speed;
                                state.angular_velocity = angular;
                                state.payload_mass = mass;
                                state.friction = friction;
                                state.roll = roll;
                                state.payload_angle = payload_angle;
                                state.obstacle_clearance = clearance;
                                for (double requested_v :
                                     requested_linear) {
                                    for (double requested_w :
                                         requested_angular) {
                                        for (double joint_1 :
                                             requested_joint_1) {
                                            for (double joint_2 :
                                                 requested_joint_2) {
                                                rwb::Action action;
                                                action.linear_velocity =
                                                    requested_v;
                                                action.angular_velocity =
                                                    requested_w;
                                                action.joint_1_target =
                                                    joint_1;
                                                action.joint_2_target =
                                                    joint_2;
                                                const auto decision =
                                                    shell.project(state,
                                                                  action);
                                                ++cases;
                                                if (decision
                                                        .recovery_required) {
                                                    ++recovery_commands;
                                                }
                                                maximum_speed_utilization =
                                                    std::max(
                                                        maximum_speed_utilization,
                                                        decision
                                                            .speed_utilization);
                                                maximum_acceleration_utilization =
                                                    std::max(
                                                        maximum_acceleration_utilization,
                                                        decision
                                                            .acceleration_utilization);
                                                maximum_lateral_utilization =
                                                    std::max(
                                                        maximum_lateral_utilization,
                                                        decision
                                                            .lateral_utilization);
                                                maximum_support_utilization =
                                                    std::max(
                                                        maximum_support_utilization,
                                                        decision
                                                            .support_utilization);
                                                const bool outside_command_envelope =
                                                    std::abs(decision.action
                                                                 .linear_velocity) >
                                                        limits
                                                                .max_linear_speed +
                                                            1e-8 ||
                                                    std::abs(decision.action
                                                                 .angular_velocity) >
                                                        limits
                                                                .max_angular_speed +
                                                            1e-8 ||
                                                    decision.action
                                                            .joint_1_target <
                                                        limits.min_joint_1 +
                                                            limits
                                                                    .joint_margin -
                                                            1e-8 ||
                                                    decision.action
                                                            .joint_1_target >
                                                        limits.max_joint_1 -
                                                            limits
                                                                    .joint_margin +
                                                            1e-8 ||
                                                    decision.action
                                                            .joint_2_target <
                                                        limits.min_joint_2 +
                                                            limits
                                                                    .joint_margin -
                                                            1e-8 ||
                                                    decision.action
                                                            .joint_2_target >
                                                        limits.max_joint_2 -
                                                            limits
                                                                    .joint_margin +
                                                            1e-8 ||
                                                    std::abs(decision.action
                                                                 .balance_command) >
                                                        limits
                                                                .max_balance_command +
                                                            1e-8;
                                                if (outside_command_envelope) {
                                                    ++counterexamples;
                                                    if (first_counterexample
                                                            .empty()) {
                                                        std::ostringstream detail;
                                                        detail
                                                            << "state v="
                                                            << speed
                                                            << " w="
                                                            << angular
                                                            << " mass="
                                                            << mass
                                                            << " friction="
                                                            << friction
                                                            << " roll="
                                                            << roll
                                                            << " payload="
                                                            << payload_angle
                                                            << " clearance="
                                                            << clearance
                                                            << " requested=["
                                                            << requested_v
                                                            << ","
                                                            << requested_w
                                                            << ","
                                                            << joint_1
                                                            << ","
                                                            << joint_2
                                                            << "] projected=["
                                                            << decision
                                                                   .action
                                                                   .linear_velocity
                                                            << ","
                                                            << decision
                                                                   .action
                                                                   .angular_velocity
                                                            << ","
                                                            << decision
                                                                   .action
                                                                   .joint_1_target
                                                            << ","
                                                            << decision
                                                                   .action
                                                                   .joint_2_target
                                                            << "] recovery="
                                                            << decision
                                                                   .recovery_required;
                                                        first_counterexample =
                                                            detail.str();
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    const std::filesystem::path path(output_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        std::cerr << "cannot write " << output_path << '\n';
        return 2;
    }
    output << std::setprecision(10);
    output << "Constructed safety shell specification\n";
    output << "control_dt " << limits.control_dt << '\n';
    output << "max_linear_speed " << limits.max_linear_speed << '\n';
    output << "max_angular_speed " << limits.max_angular_speed << '\n';
    output << "max_linear_accel " << limits.max_linear_accel << '\n';
    output << "max_angular_accel " << limits.max_angular_accel << '\n';
    output << "stop_decel " << limits.stop_decel << '\n';
    output << "joint_1 [" << limits.min_joint_1 << ", "
           << limits.max_joint_1 << "] margin "
           << limits.joint_margin << '\n';
    output << "joint_2 [" << limits.min_joint_2 << ", "
           << limits.max_joint_2 << "] margin "
           << limits.joint_margin << '\n';
    output << "support_half_width " << limits.support_half_width << '\n';
    output << "max_lateral_accel " << limits.max_lateral_accel << '\n';
    output << "epistemic_authority "
              "none_inside_body_shell\n";
    output << "discrete_cases " << cases << '\n';
    output << "counterexamples " << counterexamples << '\n';
    output << "recovery_commands " << recovery_commands << '\n';
    output << "max_speed_utilization "
           << maximum_speed_utilization << '\n';
    output << "max_acceleration_utilization "
           << maximum_acceleration_utilization << '\n';
    output << "max_lateral_utilization "
           << maximum_lateral_utilization << '\n';
    output << "max_support_utilization "
           << maximum_support_utilization << '\n';
    output << "first_counterexample "
           << (first_counterexample.empty()
                   ? "none"
                   : first_counterexample)
           << '\n';
    std::cout << "shell specification: " << cases
              << " cases, counterexamples=" << counterexamples
              << ", recovery_commands=" << recovery_commands << '\n';
    return counterexamples == 0 ? 0 : 3;
}

