#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "rwb/closed_loop.hpp"
#include "rwb/metrics.hpp"

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

void print_arm(const char* name, const rwb::AggregateMetrics& metrics) {
    std::cout << std::left << std::setw(12) << name
              << " success=" << std::fixed << std::setprecision(3)
              << metrics.success_rate
              << " irreversible=" << metrics.irreversible_event_rate
              << " shell=" << metrics.shell_trigger_rate
              << " expansion=" << metrics.boundary_expansion_rate
              << " time=" << metrics.mean_completion_time
              << " p95_us=" << metrics.p95_control_latency_us << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    const int episode_count =
        parse_int(argc, argv, "--episodes", 40);
    const std::uint64_t seed =
        parse_seed(argc, argv, "--seed", 2026);
    const std::string model_path =
        parse_string(argc, argv, "--model",
                     "models/recoverability.model");
    const std::string output_dir =
        parse_string(argc, argv, "--out", "outputs");
    bool stress = false;
    for (int index = 1; index < argc; ++index) {
        if (std::string(argv[index]) == "--stress") {
            stress = true;
        }
    }

    rwb::RuntimeConfig config;
    if (stress) {
        config.scenario.payload_mass_min = 9.0;
        config.scenario.payload_mass_max = 17.0;
        config.scenario.friction_min = 0.70;
        config.scenario.friction_max = 1.05;
        config.scenario.disturbance_amplitude = 10.0;
        config.scenario.disturbance_period = 2.15;
        config.scenario.task_irreversibility_angle = 0.43;
        config.scenario.body_irreversibility_roll = 0.29;
    }
    rwb::RecoverabilityMonitor base_monitor(config.monitor);
    std::string error;
    if (!base_monitor.load(model_path, error)) {
        std::cerr << error << '\n';
        return 2;
    }

    const std::filesystem::path output_root(output_dir);
    std::filesystem::create_directories(output_root / "telemetry");

    rwb::RecoverabilityMonitor legacy_monitor = base_monitor;
    rwb::RecoverabilityMonitor learned_monitor = base_monitor;
    rwb::ClosedLoopRunner legacy_runner(config, legacy_monitor);
    rwb::ClosedLoopRunner learned_runner(config, learned_monitor);

    std::vector<rwb::EpisodeResult> legacy_results;
    std::vector<rwb::EpisodeResult> learned_results;
    legacy_results.reserve(episode_count);
    learned_results.reserve(episode_count);

    for (int episode = 0; episode < episode_count; ++episode) {
        const std::uint64_t episode_seed =
            seed + static_cast<std::uint64_t>(episode) * 7919U;
        legacy_results.push_back(legacy_runner.run(
            rwb::ControlMode::Legacy, episode_seed,
            (output_root / "telemetry" /
             ("legacy_" + std::to_string(episode) + ".csv"))
                .string()));
        learned_results.push_back(learned_runner.run(
            rwb::ControlMode::Reversible, episode_seed,
            (output_root / "telemetry" /
             ("reversible_" + std::to_string(episode) + ".csv"))
                .string()));
    }

    const rwb::AggregateMetrics legacy =
        rwb::aggregate_metrics(legacy_results);
    const rwb::AggregateMetrics reversible =
        rwb::aggregate_metrics(learned_results);

    rwb::write_episode_csv((output_root / "legacy_episodes.csv").string(),
                           legacy_results);
    rwb::write_episode_csv(
        (output_root / "reversible_episodes.csv").string(),
        learned_results);
    rwb::write_summary_json((output_root / "summary.json").string(),
                            legacy, reversible,
                            base_monitor.training_sample_count());
    rwb::write_comparison_svg(
        (output_root / "comparison.svg").string(), legacy, reversible);
    rwb::write_markdown_report(
        (output_root / "report.md").string(), legacy, reversible,
        base_monitor.training_sample_count());

    print_arm("legacy", legacy);
    print_arm("reversible", reversible);
    std::cout << "boundary cells="
              << learned_runner.boundary_book().cell_count()
              << " positive="
              << learned_runner.boundary_book().positive_certificates()
              << " negative="
              << learned_runner.boundary_book().negative_certificates()
              << '\n';
    std::cout << "profile: " << (stress ? "stress" : "standard")
              << '\n';
    std::cout << "artifacts: " << output_dir << '\n';
    return 0;
}

