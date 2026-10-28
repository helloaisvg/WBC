#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include "rwb/dynamics.hpp"
#include "rwb/monitor.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "failure: " << message << '\n';
        ++failures;
    }
}

}  // namespace

int main() {
    rwb::ShellLimits limits;
    rwb::SafetyShell shell(limits);
    shell.freeze();
    expect(shell.frozen(), "shell freezes before control");

    rwb::RobotState state;
    state.linear_velocity = 0.55;
    state.angular_velocity = 0.30;
    state.obstacle_clearance = 0.72;
    rwb::Action extreme;
    extreme.linear_velocity = 3.0;
    extreme.angular_velocity = -4.0;
    extreme.joint_1_target = 8.0;
    extreme.joint_2_target = -2.0;
    const rwb::ShellDecision projected = shell.project(state, extreme);
    expect(std::abs(projected.action.linear_velocity) <=
               limits.max_linear_speed,
           "linear command remains in envelope");
    expect(std::abs(projected.action.angular_velocity) <=
               limits.max_angular_speed,
           "angular command remains in envelope");
    expect(projected.action.joint_1_target <=
               limits.max_joint_1 - limits.joint_margin + 1e-9,
           "joint one remains inside hard margin");
    expect(projected.action.joint_2_target >=
               limits.min_joint_2 + limits.joint_margin - 1e-9,
           "joint two remains inside hard margin");

    rwb::RuntimeConfig config;
    rwb::WarehouseSim simulator(config.scenario);
    simulator.reset(91);
    const double initial_x = simulator.state().position.x;
    rwb::Action forward;
    forward.linear_velocity = 0.2;
    simulator.step(forward);
    expect(simulator.state().position.x > initial_x,
           "simulator advances under forward control");

    rwb::RecoverabilityMonitor monitor(config.monitor);
    rwb::Rng rng(77);
    for (int sample = 0; sample < 160; ++sample) {
        rwb::FeatureVector features{};
        for (double& value : features) {
            value = rng.uniform(0.0, 1.0);
        }
        const bool label =
            features[0] + features[8] + features[9] < 1.55;
        monitor.add_training_sample(features, label);
    }
    const auto result = monitor.train(4, 3, 60);
    expect(result.positive > 0 && result.negative > 0,
           "synthetic monitor has both labels");
    expect(monitor.trained(), "monitor trains an ensemble");

    const auto first = monitor.predict(rwb::FeatureVector{});
    const auto second =
        monitor.predict(rwb::FeatureVector{{1, 1, 1, 1, 1, 1, 1, 1, 1,
                                            1, 1, 1, 1, 1, 1, 1, 1}});
    expect(first.rescue_probability > second.rescue_probability,
           "monitor learns the risk direction");

    const auto path =
        std::filesystem::temp_directory_path() /
        "reversible_wbc_monitor_test.model";
    std::string error;
    expect(monitor.save(path.string(), error), "monitor saves");
    rwb::RecoverabilityMonitor loaded(config.monitor);
    expect(loaded.load(path.string(), error), "monitor loads");
    const auto reloaded =
        loaded.predict(rwb::FeatureVector{{0.2, 0.3, 0.2, 0.1, 0.4,
                                          0.6, 0.5, 0.2, 0.1, 0.1,
                                          0.2, 0.3, 0.4, 0.5, 0.4, 0.6,
                                          0.2}});
    const auto original =
        monitor.predict(rwb::FeatureVector{{0.2, 0.3, 0.2, 0.1, 0.4,
                                           0.6, 0.5, 0.2, 0.1, 0.1,
                                           0.2, 0.3, 0.4, 0.5, 0.4, 0.6,
                                           0.2}});
    expect(std::abs(reloaded.rescue_probability -
                    original.rescue_probability) < 1e-12,
           "monitor round trip preserves inference");

    std::filesystem::remove(path);
    return failures == 0 ? 0 : 1;
}

