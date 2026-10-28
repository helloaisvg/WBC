#include "rwb/dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace rwb {
namespace {

Vec2 base_forward(const RobotState& state) {
    return {std::cos(state.yaw), std::sin(state.yaw)};
}

Vec2 base_left(const RobotState& state) {
    return {-std::sin(state.yaw), std::cos(state.yaw)};
}

double arm_forward_offset(const RobotState& state) {
    return 0.08 + 0.31 * std::cos(state.joint_1) +
           0.28 * std::cos(state.joint_1 + state.joint_2);
}

double arm_lateral_offset(const RobotState& state) {
    return 0.31 * std::sin(state.joint_1) +
           0.28 * std::sin(state.joint_1 + state.joint_2);
}

double map_boundary_clearance(const RobotState& state,
                              const ScenarioConfig& config) {
    return std::min({state.position.x - config.map_min_x,
                     config.map_max_x - state.position.x,
                     state.position.y - config.map_min_y,
                     config.map_max_y - state.position.y});
}

}  // namespace

WarehouseSim::WarehouseSim(ScenarioConfig config)
    : config_(std::move(config)) {
    reset(1);
}

void WarehouseSim::reset(std::uint64_t seed) {
    seed_ = seed;
    Rng rng(seed);
    state_ = RobotState{};
    state_.position = {0.58, -1.55};
    state_.yaw = 0.0;
    state_.joint_1 = -0.04;
    state_.joint_2 = 1.12;
    state_.payload_mass =
        rng.uniform(config_.payload_mass_min, config_.payload_mass_max);
    state_.friction =
        rng.uniform(config_.friction_min, config_.friction_max);
    disturbance_phase_ = rng.uniform(0.0, config_.disturbance_period);
    time_ = 0.0;
    waypoint_index_ = 0;
    finished_ = false;
    event_ = IrreversibleEvent::None;
    update_clearance();
}

void WarehouseSim::set_state(const RobotState& state) {
    state_ = state;
    update_clearance();
}

Vec2 WarehouseSim::current_waypoint() const {
    if (waypoint_index_ >=
        static_cast<int>(config_.waypoints.size())) {
        return config_.waypoints.back();
    }
    return config_.waypoints[static_cast<std::size_t>(waypoint_index_)];
}

double WarehouseSim::current_goal_distance() const {
    return (current_waypoint() - state_.position).norm();
}

double WarehouseSim::external_disturbance_at(double time) const {
    const double phase =
        std::fmod(time + disturbance_phase_, config_.disturbance_period);
    const double onset = 0.23;
    const double width = 0.31;
    if (phase < onset || phase > onset + width) {
        return 0.0;
    }
    const double envelope =
        std::sin(kPi * (phase - onset) / width);
    const double direction =
        std::sin(1.7 * (time + disturbance_phase_)) >= 0.0 ? 1.0 : -1.0;
    return direction * config_.disturbance_amplitude * envelope;
}

void WarehouseSim::integrate(double dt, const Action& action) {
    const double previous_velocity = state_.linear_velocity;
    const double velocity_alpha =
        1.0 - std::exp(-dt / config_.velocity_time_constant);
    const double yaw_alpha =
        1.0 - std::exp(-dt / config_.yaw_rate_time_constant);
    state_.linear_velocity +=
        (action.linear_velocity - state_.linear_velocity) * velocity_alpha;
    state_.angular_velocity +=
        (action.angular_velocity - state_.angular_velocity) * yaw_alpha;
    state_.longitudinal_accel =
        (state_.linear_velocity - previous_velocity) / dt;

    state_.joint_velocity_1 +=
        (config_.arm_stiffness *
             (action.joint_1_target - state_.joint_1) -
         config_.arm_damping * state_.joint_velocity_1) *
        dt;
    state_.joint_velocity_2 +=
        (config_.arm_stiffness *
             (action.joint_2_target - state_.joint_2) -
         config_.arm_damping * state_.joint_velocity_2) *
        dt;
    state_.joint_1 += state_.joint_velocity_1 * dt;
    state_.joint_2 += state_.joint_velocity_2 * dt;

    state_.joint_1 = clamp(state_.joint_1, -1.10, 0.95);
    state_.joint_2 = clamp(state_.joint_2, 0.05, 2.30);

    state_.yaw =
        wrap_angle(state_.yaw + state_.angular_velocity * dt);
    const Vec2 forward = base_forward(state_);
    state_.position += forward * (state_.linear_velocity * dt);

    const double extension = clamp01(
        0.55 + 0.55 * (arm_forward_offset(state_) - 0.12) / 0.40);
    const double external = external_disturbance_at(time_ + dt);
    state_.external_disturbance = external;
    const double lateral_accel =
        state_.linear_velocity * state_.angular_velocity +
        0.58 * external +
        config_.balance_authority * action.balance_command;
    const double effective_mass =
        (config_.base_mass + state_.payload_mass) /
        config_.base_mass;

    const double roll_accel =
        -config_.roll_omega * config_.roll_omega * state_.roll -
        2.0 * config_.roll_damping * config_.roll_omega *
            state_.roll_rate +
        config_.load_coupling * effective_mass * (1.0 + 0.20 * extension) *
            lateral_accel +
        config_.balance_authority * action.balance_command;
    state_.roll_rate += roll_accel * dt;
    state_.roll += state_.roll_rate * dt;

    const double payload_excitation =
        state_.longitudinal_accel + 0.58 * lateral_accel;
    const double pendulum_accel =
        -9.81 / config_.payload_pendulum_length *
            std::sin(state_.payload_angle) -
        config_.payload_damping * state_.payload_rate -
        (0.70 + 0.10 * effective_mass) * payload_excitation /
            config_.payload_pendulum_length;
    state_.payload_rate += pendulum_accel * dt;
    state_.payload_angle += state_.payload_rate * dt;

    time_ += dt;
    update_clearance();
    detect_events();
    update_task_state();
}

void WarehouseSim::step(const Action& action) {
    if (finished_ || irreversible()) {
        return;
    }
    const double substep_dt =
        config_.control_dt / static_cast<double>(config_.substeps);
    for (int substep = 0; substep < config_.substeps; ++substep) {
        integrate(substep_dt, action);
        if (irreversible()) {
            break;
        }
    }
}

double WarehouseSim::minimum_clearance() const {
    const Vec2 forward = base_forward(state_);
    const Vec2 left = base_left(state_);
    const Vec2 end_effector =
        state_.position + forward * arm_forward_offset(state_) +
        left * arm_lateral_offset(state_);
    const Vec2 payload =
        end_effector + forward * 0.13;

    double clearance = map_boundary_clearance(state_, config_) - 0.30;
    for (const Rect& obstacle : config_.obstacles) {
        clearance =
            std::min(clearance,
                     point_rect_distance(state_.position, obstacle) - 0.30);
        clearance =
            std::min(clearance,
                     point_rect_distance(end_effector, obstacle) - 0.075);
        clearance =
            std::min(clearance,
                     point_rect_distance(payload, obstacle) - 0.19);
    }
    return clearance;
}

void WarehouseSim::update_clearance() {
    state_.obstacle_clearance = minimum_clearance();
}

void WarehouseSim::detect_events() {
    if (std::abs(state_.payload_angle) >
        config_.task_irreversibility_angle) {
        event_ = IrreversibleEvent::PayloadFall;
        return;
    }
    if (std::abs(state_.roll) > config_.body_irreversibility_roll) {
        event_ = IrreversibleEvent::BodyFall;
        return;
    }
    if (std::abs(state_.payload_angle) > 1.25 ||
        std::abs(state_.roll) > 1.05) {
        event_ = IrreversibleEvent::BodyFall;
        return;
    }
    if (state_.obstacle_clearance < 0.0) {
        event_ = IrreversibleEvent::Collision;
        return;
    }
    if (state_.position.x < config_.map_min_x ||
        state_.position.x > config_.map_max_x ||
        state_.position.y < config_.map_min_y ||
        state_.position.y > config_.map_max_y) {
        event_ = IrreversibleEvent::MapExit;
    }
}

void WarehouseSim::update_task_state() {
    if (finished_ || irreversible()) {
        return;
    }
    const double distance = current_goal_distance();
    if (distance <= config_.goal_tolerance) {
        ++waypoint_index_;
        if (waypoint_index_ >=
            static_cast<int>(config_.waypoints.size())) {
            waypoint_index_ =
                static_cast<int>(config_.waypoints.size()) - 1;
            finished_ = true;
        }
    }
}

bool WarehouseSim::assess_recoverability(
    const RobotState& starting_state, const Action& action,
    std::uint64_t seed) const {
    WarehouseSim trial(*this);
    trial.state_ = starting_state;
    trial.event_ = IrreversibleEvent::None;
    trial.finished_ = false;
    trial.time_ = time_;
    trial.disturbance_phase_ =
        static_cast<double>(seed % 100003U) / 100003.0 *
        config_.disturbance_period;
    trial.update_clearance();

    constexpr double action_duration = 0.24;
    constexpr double recovery_duration = 2.30;
    const double integration_dt = 0.005;

    for (double elapsed = 0.0; elapsed < action_duration;
         elapsed += integration_dt) {
        trial.integrate(integration_dt, action);
        if (trial.irreversible()) {
            return false;
        }
    }

    SafetyShell shell(ShellLimits{});
    shell.freeze();
    for (double elapsed = 0.0; elapsed < recovery_duration;
         elapsed += integration_dt) {
        const Action recovery = shell.recovery_command(trial.state_);
        trial.integrate(integration_dt, recovery);
        if (trial.irreversible()) {
            return false;
        }
    }

    const bool stable =
        std::abs(trial.state_.linear_velocity) < 0.075 &&
        std::abs(trial.state_.angular_velocity) < 0.12 &&
        std::abs(trial.state_.roll) < 0.085 &&
        std::abs(trial.state_.payload_angle) < 0.14 &&
        std::abs(trial.state_.payload_rate) < 0.32 &&
        trial.minimum_clearance() > 0.0;
    return stable;
}

}  // namespace rwb

