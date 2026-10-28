#pragma once

#include <array>
#include <string>
#include <vector>

#include "rwb/math.hpp"

namespace rwb {

struct ShellLimits {
    double control_dt = 0.01;

    double max_linear_speed = 0.92;
    double max_angular_speed = 1.20;
    double max_linear_accel = 1.10;
    double max_angular_accel = 1.80;
    double stop_decel = 1.55;
    double linear_response_time = 0.23;
    double angular_response_time = 0.20;

    double min_joint_1 = -1.10;
    double max_joint_1 = 0.95;
    double min_joint_2 = 0.05;
    double max_joint_2 = 2.30;
    double joint_margin = 0.055;

    double base_radius = 0.32;
    double collision_margin = 0.075;
    double support_half_width = 0.36;
    double com_height = 0.64;
    double max_lateral_accel = 1.28;
    double max_roll = 0.22;
    double payload_support_half_width = 0.19;
    double payload_fall_angle = 0.52;
    double max_balance_command = 1.0;
    double balance_authority = 3.8;

    double strict_recoverability_floor = 0.50;
    double epistemic_ceiling = 0.24;
    double exploration_probability_floor = 0.50;
    double shell_trigger_margin = 0.035;
};

struct MonitorLimits {
    double learning_rate = 0.018;
    double regularization = 0.0015;
    double online_update_fraction = 0.03;
    double max_online_shift = 0.16;
    double boundary_low = 0.58;
    double boundary_high = 0.84;
};

struct ExplorationConfig {
    int probe_period_steps = 85;
    double candidate_scales[5] = {0.82, 0.94, 1.04, 1.13, 1.21};
    double target_probability = 0.71;
    double uncertainty_weight = 0.85;
    double boundary_weight = 1.25;
};

struct ScenarioConfig {
    int substeps = 2;
    int max_steps = 6200;
    double control_dt = 0.01;
    double goal_tolerance = 0.28;

    double base_length = 0.52;
    double base_width = 0.40;
    double base_mass = 58.0;
    double payload_mass_min = 4.0;
    double payload_mass_max = 13.0;
    double friction_min = 0.86;
    double friction_max = 1.35;

    double velocity_time_constant = 0.23;
    double yaw_rate_time_constant = 0.20;
    double arm_damping = 5.2;
    double arm_stiffness = 22.0;
    double payload_pendulum_length = 0.52;
    double payload_damping = 0.58;

    double roll_omega = 2.75;
    double roll_damping = 0.42;
    double load_coupling = 0.37;
    double balance_authority = 3.80;
    double disturbance_period = 2.8;
    double disturbance_amplitude = 0.44;

    double exploration_risk_floor = 0.68;
    double task_irreversibility_angle = 0.52;
    double body_irreversibility_roll = 0.31;

    double map_min_x = -0.5;
    double map_max_x = 11.5;
    double map_min_y = -3.0;
    double map_max_y = 3.0;
    std::vector<Rect> obstacles;
    std::vector<Vec2> waypoints;
    bool output_step_telemetry = true;

    ScenarioConfig();
    static ScenarioConfig warehouse();
};

struct RuntimeConfig {
    ShellLimits shell;
    MonitorLimits monitor;
    ExplorationConfig exploration;
    ScenarioConfig scenario;
};

}  // namespace rwb

