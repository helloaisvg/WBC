#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "rwb/config.hpp"

namespace rwb {

struct Action {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    double joint_1_target = 0.0;
    double joint_2_target = 1.10;
    double balance_command = 0.0;
};

struct RobotState {
    Vec2 position;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    double joint_1 = 0.0;
    double joint_2 = 1.10;
    double joint_velocity_1 = 0.0;
    double joint_velocity_2 = 0.0;

    double roll = 0.0;
    double roll_rate = 0.0;
    double payload_angle = 0.0;
    double payload_rate = 0.0;
    double payload_mass = 8.0;
    double friction = 1.0;

    double external_disturbance = 0.0;
    double longitudinal_accel = 0.0;
    double obstacle_clearance = 5.0;
};

enum class ShellReason : std::uint32_t {
    None = 0,
    Speed = 1U << 0U,
    Acceleration = 1U << 1U,
    LateralStability = 1U << 2U,
    JointRange = 1U << 3U,
    StoppingDistance = 1U << 4U,
    BodyLimit = 1U << 5U,
};

inline ShellReason operator|(ShellReason left, ShellReason right) {
    return static_cast<ShellReason>(
        static_cast<std::uint32_t>(left) | static_cast<std::uint32_t>(right));
}

inline ShellReason& operator|=(ShellReason& left, ShellReason right) {
    left = left | right;
    return left;
}

inline bool has_reason(ShellReason value, ShellReason flag) {
    return (static_cast<std::uint32_t>(value) & static_cast<std::uint32_t>(flag)) != 0U;
}

struct ShellDecision {
    Action action;
    bool triggered = false;
    bool critical_triggered = false;
    ShellReason reason = ShellReason::None;
    double speed_utilization = 0.0;
    double acceleration_utilization = 0.0;
    double lateral_utilization = 0.0;
    double support_utilization = 0.0;
    double joint_utilization = 0.0;
    double stopping_utilization = 0.0;
    bool recovery_required = false;
};

struct MonitorPrediction {
    double rescue_probability = 0.0;
    double epistemic_uncertainty = 1.0;
    double coverage = 0.0;
    double strict_threshold = 1.0;
    double exploration_threshold = 1.0;
    std::array<double, 5> components{};
};

struct GateDecision {
    Action action;
    MonitorPrediction prediction;
    bool replace_with_recovery = false;
    bool governor_scaled = false;
    bool exploration_probe = false;
};

enum class IrreversibleEvent : std::uint8_t {
    None = 0,
    PayloadFall = 1,
    BodyFall = 2,
    Collision = 3,
    MapExit = 4,
};

struct StepTelemetry {
    int step = 0;
    double time = 0.0;
    Vec2 position;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    double roll = 0.0;
    double payload_angle = 0.0;
    double rescue_probability = 0.0;
    double epistemic_uncertainty = 0.0;
    bool shell_triggered = false;
    bool gate_triggered = false;
    bool exploration_probe = false;
    bool boundary_expanded = false;
    IrreversibleEvent event = IrreversibleEvent::None;
};

struct EpisodeResult {
    int seed = 0;
    int steps = 0;
    double elapsed_sim_time = 0.0;
    bool reached_goal = false;
    bool irreversible = false;
    IrreversibleEvent event = IrreversibleEvent::None;
    int shell_triggers = 0;
    int gate_triggers = 0;
    int recovery_actions = 0;
    int exploratory_interactions = 0;
    int boundary_expansions = 0;
    double final_goal_distance = 0.0;
    double payload_mass = 0.0;
    double friction = 0.0;
    double mean_rescue_probability = 0.0;
    double mean_epistemic_uncertainty = 0.0;
    double mean_control_latency_us = 0.0;
    double p95_control_latency_us = 0.0;
};

inline const char* event_name(IrreversibleEvent event) {
    switch (event) {
        case IrreversibleEvent::PayloadFall:
            return "payload_fall";
        case IrreversibleEvent::BodyFall:
            return "body_fall";
        case IrreversibleEvent::Collision:
            return "collision";
        case IrreversibleEvent::MapExit:
            return "map_exit";
        case IrreversibleEvent::None:
        default:
            return "none";
    }
}

}  // namespace rwb

