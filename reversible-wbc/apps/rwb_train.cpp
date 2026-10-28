#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rwb/dynamics.hpp"
#include "rwb/monitor.hpp"
#include "rwb/policy.hpp"

namespace {

int parse_int(int argc, char** argv, const std::string& name,
              int fallback) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return std::stoi(argv[index + 1]);
        }
    }
    return fallback;
}

std::uint64_t parse_seed(int argc, char** argv, const std::string& name,
                         std::uint64_t fallback) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return std::stoull(argv[index + 1]);
        }
    }
    return fallback;
}

std::string parse_string(int argc, char** argv, const std::string& name,
                         const std::string& fallback) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return argv[index + 1];
        }
    }
    return fallback;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::vector<std::pair<rwb::RobotState, rwb::Action>> load_curriculum(
    const std::string& path) {
    std::vector<std::pair<rwb::RobotState, rwb::Action>> samples;
    if (path.empty()) {
        return samples;
    }
    std::ifstream input(path);
    std::string line;
    if (!input || !std::getline(input, line)) {
        return samples;
    }
    const std::vector<std::string> header = split_csv(line);
    std::unordered_map<std::string, std::size_t> column;
    for (std::size_t index = 0; index < header.size(); ++index) {
        column[header[index]] = index;
    }
    auto value = [&column](const std::vector<std::string>& fields,
                           const std::string& name) {
        return std::stod(fields.at(column.at(name)));
    };
    while (std::getline(input, line)) {
        const std::vector<std::string> fields = split_csv(line);
        if (fields.size() != header.size()) {
            continue;
        }
        rwb::RobotState state;
        state.position = {value(fields, "x"), value(fields, "y")};
        state.yaw = value(fields, "yaw");
        state.linear_velocity = value(fields, "v");
        state.angular_velocity = value(fields, "w");
        state.roll = value(fields, "roll");
        state.payload_angle = value(fields, "payload_angle");
        state.payload_rate = value(fields, "payload_rate");
        state.joint_1 = value(fields, "joint_1");
        state.joint_2 = value(fields, "joint_2");
        state.joint_velocity_1 = value(fields, "joint_velocity_1");
        state.joint_velocity_2 = value(fields, "joint_velocity_2");
        state.payload_mass = value(fields, "payload_mass");
        state.friction = value(fields, "friction");
        state.obstacle_clearance = value(fields, "clearance");
        state.external_disturbance =
            value(fields, "external_disturbance");
        rwb::Action action;
        action.linear_velocity = value(fields, "requested_v");
        action.angular_velocity = value(fields, "requested_w");
        action.joint_1_target = value(fields, "requested_joint_1");
        action.joint_2_target = value(fields, "requested_joint_2");
        action.balance_command = value(fields, "requested_balance");
        samples.emplace_back(state, action);
    }
    return samples;
}

}  // namespace

int main(int argc, char** argv) {
    const int requested_samples =
        parse_int(argc, argv, "--samples", 24000);
    const int policy_samples =
        parse_int(argc, argv, "--policy-samples", 8000);
    const std::uint64_t seed =
        parse_seed(argc, argv, "--seed", 17);
    const std::string output_path =
        parse_string(argc, argv, "--output",
                     "models/recoverability.model");
    const std::string curriculum_path =
        parse_string(argc, argv, "--curriculum", "");
    bool stress = false;
    bool mixed = false;
    for (int index = 1; index < argc; ++index) {
        if (std::string(argv[index]) == "--stress") {
            stress = true;
        } else if (std::string(argv[index]) == "--mixed") {
            mixed = true;
        }
    }

    rwb::RuntimeConfig config;
    if (stress && !mixed) {
        config.scenario.payload_mass_min = 9.0;
        config.scenario.payload_mass_max = 17.0;
        config.scenario.friction_min = 0.70;
        config.scenario.friction_max = 1.05;
        config.scenario.disturbance_amplitude = 10.0;
        config.scenario.disturbance_period = 2.15;
        config.scenario.task_irreversibility_angle = 0.43;
        config.scenario.body_irreversibility_roll = 0.29;
    }
    rwb::WarehouseSim simulator(config.scenario);
    simulator.reset(seed);
    rwb::ScenarioConfig curriculum_scenario = config.scenario;
    if (stress && mixed) {
        curriculum_scenario.payload_mass_min = 9.0;
        curriculum_scenario.payload_mass_max = 17.0;
        curriculum_scenario.friction_min = 0.70;
        curriculum_scenario.friction_max = 1.05;
        curriculum_scenario.disturbance_amplitude = 10.0;
        curriculum_scenario.disturbance_period = 2.15;
        curriculum_scenario.task_irreversibility_angle = 0.43;
        curriculum_scenario.body_irreversibility_roll = 0.29;
    }
    rwb::WarehouseSim curriculum_simulator(curriculum_scenario);
    curriculum_simulator.reset(seed + 7001U);
    rwb::RecoverabilityMonitor monitor(config.monitor);
    rwb::SafetyShell shell(config.shell);
    shell.freeze();
    rwb::Rng rng(seed);

    for (int sample = 0; sample < requested_samples; ++sample) {
        rwb::RobotState state;
        state.position = {rng.uniform(0.72, 1.42),
                          rng.uniform(-2.20, 2.20)};
        state.yaw = rng.uniform(-rwb::kPi, rwb::kPi);
        state.linear_velocity = rng.uniform(-0.28, 1.02);
        state.angular_velocity = rng.uniform(-1.28, 1.28);
        state.joint_1 = rng.uniform(-0.92, 0.82);
        state.joint_2 = rng.uniform(0.16, 2.18);
        state.joint_velocity_1 = rng.uniform(-2.35, 2.35);
        state.joint_velocity_2 = rng.uniform(-2.35, 2.35);
        state.roll = rng.uniform(-0.20, 0.20);
        state.roll_rate = rng.uniform(-0.52, 0.52);
        state.payload_angle = rng.uniform(-0.33, 0.33);
        state.payload_rate = rng.uniform(-0.95, 0.95);
        state.payload_mass = rng.uniform(
            config.scenario.payload_mass_min,
            config.scenario.payload_mass_max);
        state.friction = rng.uniform(config.scenario.friction_min,
                                     config.scenario.friction_max);
        state.obstacle_clearance = 1.20;
        simulator.set_state(state);
        state = simulator.state();

        rwb::Action action;
        action.linear_velocity = rng.uniform(-0.24, 1.16);
        action.angular_velocity = rng.uniform(-1.48, 1.48);
        action.joint_1_target = rng.uniform(-1.05, 0.90);
        action.joint_2_target = rng.uniform(0.08, 2.28);

        const rwb::ShellDecision projected = shell.project(state, action);
        const bool recovered = simulator.assess_recoverability(
            state, projected.action, rng.raw());
        const rwb::FeatureVector features =
            rwb::extract_features(state, projected.action, projected, shell);
        monitor.add_training_sample(features, recovered);
    }

    rwb::NominalPlanner planner;
    int collected_policy_samples = 0;
    int rollout_index = 0;
    while (collected_policy_samples < policy_samples &&
           rollout_index < 256) {
        rwb::WarehouseSim rollout(config.scenario);
        rollout.reset(seed + 1009U *
                              static_cast<std::uint64_t>(rollout_index + 1));
        ++rollout_index;
        for (int step = 0;
             step < config.scenario.max_steps &&
             collected_policy_samples < policy_samples &&
             !rollout.finished() && !rollout.irreversible();
             ++step) {
            const rwb::RobotState state = rollout.state();
            const rwb::Action nominal =
                planner.command(state, rollout.current_waypoint(),
                                rollout.time());
            const rwb::ShellDecision projected =
                shell.project(state, nominal);
            const rwb::FeatureVector features = rwb::extract_features(
                state, projected.action, projected, shell);
            const bool recovered = rollout.assess_recoverability(
                state, projected.action, rng.raw());
            monitor.add_training_sample(features, recovered);
            ++collected_policy_samples;
            rollout.step(projected.action);
        }
    }

    const auto curriculum_samples = load_curriculum(curriculum_path);
    for (const auto& [state, action] : curriculum_samples) {
        const rwb::ShellDecision projected = shell.project(state, action);
        const bool recovered = curriculum_simulator.assess_recoverability(
            state, projected.action, rng.raw());
        const rwb::FeatureVector features =
            rwb::extract_features(state, projected.action, projected, shell);
        monitor.add_training_sample(features, recovered);
    }

    const rwb::MonitorTrainingResult training =
        monitor.train(seed + 31U, 5, 260);
    if (training.samples != requested_samples + collected_policy_samples +
                                  static_cast<int>(curriculum_samples.size()) ||
        training.positive == 0 || training.negative == 0) {
        std::cerr << "training distribution has no separable labels: "
                  << training.positive << " positive, "
                  << training.negative << " negative\n";
        return 2;
    }

    const std::filesystem::path model_path(output_path);
    if (model_path.has_parent_path()) {
        std::filesystem::create_directories(model_path.parent_path());
    }
    std::string error;
    if (!monitor.save(output_path, error)) {
        std::cerr << error << '\n';
        return 3;
    }

    std::filesystem::path report_path =
        model_path.parent_path() / "training_report.txt";
    std::ofstream report(report_path);
    if (!report) {
        std::cerr << "cannot write " << report_path << '\n';
        return 4;
    }
    report << "samples " << training.samples << '\n';
    report << "random_samples " << requested_samples << '\n';
    report << "policy_samples " << collected_policy_samples << '\n';
    report << "curriculum_samples " << curriculum_samples.size() << '\n';
    report << "mixed_training " << (mixed ? "true" : "false") << '\n';
    report << "positive " << training.positive << '\n';
    report << "negative " << training.negative << '\n';
    report << "ensemble 5\n";
    report << "epochs 260\n";
    report << "final_loss " << training.final_loss << '\n';
    report << "mean_confidence " << training.mean_confidence << '\n';
    report << "brier_score " << training.brier_score << '\n';
    report << "auc " << training.auc << '\n';
    report << "positive_accuracy " << training.positive_accuracy << '\n';
    report << "negative_accuracy " << training.negative_accuracy << '\n';
    report << "strict_threshold " << monitor.strict_threshold() << '\n';
    report << "exploration_threshold "
           << monitor.exploration_threshold() << '\n';

    std::cout << "trained recoverability monitor: "
              << training.samples << " trials, " << training.positive
              << " recoverable, " << training.negative
              << " irreversible, confidence "
              << training.mean_confidence << ", AUC " << training.auc
              << "\n";
    std::cout << "model: " << output_path << '\n';
    return 0;
}

