#pragma once

#include <cstdint>
#include <vector>

#include "rwb/safety_shell.hpp"

namespace rwb {

class WarehouseSim {
public:
    explicit WarehouseSim(ScenarioConfig config);

    void reset(std::uint64_t seed);
    void step(const Action& action);
    bool assess_recoverability(const RobotState& state, const Action& action,
                               std::uint64_t seed) const;

    void set_state(const RobotState& state);
    const RobotState& state() const { return state_; }
    const ScenarioConfig& config() const { return config_; }
    const std::vector<Rect>& obstacles() const { return config_.obstacles; }

    bool finished() const { return finished_; }
    bool irreversible() const { return event_ != IrreversibleEvent::None; }
    IrreversibleEvent event() const { return event_; }
    int waypoint_index() const { return waypoint_index_; }
    Vec2 current_waypoint() const;
    double current_goal_distance() const;
    double time() const { return time_; }
    std::uint64_t seed() const { return seed_; }

private:
    ScenarioConfig config_;
    RobotState state_;
    std::uint64_t seed_ = 0;
    double time_ = 0.0;
    double disturbance_phase_ = 0.0;
    int waypoint_index_ = 0;
    bool finished_ = false;
    IrreversibleEvent event_ = IrreversibleEvent::None;

    void integrate(double dt, const Action& action);
    void update_clearance();
    void detect_events();
    void update_task_state();
    double external_disturbance_at(double time) const;
    double minimum_clearance() const;
};

}  // namespace rwb

