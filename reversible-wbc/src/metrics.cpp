#include "rwb/metrics.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace rwb {
namespace {

void require_stream(const std::ofstream& output, const std::string& path) {
    if (!output) {
        throw std::runtime_error("cannot write " + path);
    }
}

std::string percent(double value) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(2) << value * 100.0 << '%';
    return output.str();
}

}  // namespace

AggregateMetrics aggregate_metrics(
    const std::vector<EpisodeResult>& episodes) {
    AggregateMetrics result;
    result.episodes = static_cast<int>(episodes.size());
    if (episodes.empty()) {
        return result;
    }

    double time_sum = 0.0;
    double distance_sum = 0.0;
    double probability_sum = 0.0;
    double uncertainty_sum = 0.0;
    double latency_sum = 0.0;
    for (const EpisodeResult& episode : episodes) {
        result.successful_episodes += episode.reached_goal ? 1 : 0;
        result.irreversible_episodes += episode.irreversible ? 1 : 0;
        result.total_steps += episode.steps;
        result.shell_triggers += episode.shell_triggers;
        result.gate_triggers += episode.gate_triggers;
        result.exploratory_interactions +=
            episode.exploratory_interactions;
        result.boundary_expansions += episode.boundary_expansions;
        if (episode.reached_goal) {
            time_sum += episode.elapsed_sim_time;
        }
        distance_sum += episode.final_goal_distance;
        probability_sum += episode.mean_rescue_probability;
        uncertainty_sum += episode.mean_epistemic_uncertainty;
        latency_sum += episode.mean_control_latency_us;
        result.p95_control_latency_us =
            std::max(result.p95_control_latency_us,
                     episode.p95_control_latency_us);
    }

    const double inverse_episodes =
        1.0 / static_cast<double>(episodes.size());
    result.success_rate =
        static_cast<double>(result.successful_episodes) * inverse_episodes;
    result.irreversible_event_rate =
        static_cast<double>(result.irreversible_episodes) *
        inverse_episodes;
    result.shell_trigger_rate =
        static_cast<double>(result.shell_triggers) /
        std::max(1LL, result.total_steps);
    result.boundary_expansion_rate =
        static_cast<double>(result.boundary_expansions) /
        std::max(1LL, result.exploratory_interactions);
    result.mean_completion_time =
        result.successful_episodes > 0
            ? time_sum / static_cast<double>(result.successful_episodes)
            : 0.0;
    result.mean_final_goal_distance = distance_sum * inverse_episodes;
    result.mean_rescue_probability = probability_sum * inverse_episodes;
    result.mean_epistemic_uncertainty =
        uncertainty_sum * inverse_episodes;
    result.mean_control_latency_us = latency_sum * inverse_episodes;
    return result;
}

void write_episode_csv(const std::string& path,
                       const std::vector<EpisodeResult>& episodes) {
    std::ofstream output(path);
    require_stream(output, path);
    output << "seed,steps,elapsed_sim_time,reached_goal,irreversible,"
              "event,shell_triggers,gate_triggers,recovery_actions,"
              "exploratory_interactions,boundary_expansions,"
              "final_goal_distance,payload_mass,friction,"
              "mean_rescue_probability,mean_epistemic_uncertainty,"
              "mean_control_latency_us,p95_control_latency_us\n";
    for (const EpisodeResult& episode : episodes) {
        output << episode.seed << ',' << episode.steps << ','
               << episode.elapsed_sim_time << ','
               << (episode.reached_goal ? 1 : 0) << ','
               << (episode.irreversible ? 1 : 0) << ','
               << event_name(episode.event) << ','
               << episode.shell_triggers << ',' << episode.gate_triggers
               << ',' << episode.recovery_actions << ','
               << episode.exploratory_interactions << ','
               << episode.boundary_expansions << ','
               << episode.final_goal_distance << ','
               << episode.payload_mass << ',' << episode.friction << ','
               << episode.mean_rescue_probability << ','
               << episode.mean_epistemic_uncertainty << ','
               << episode.mean_control_latency_us << ','
               << episode.p95_control_latency_us << '\n';
    }
}

void write_summary_json(const std::string& path,
                        const AggregateMetrics& legacy,
                        const AggregateMetrics& reversible,
                        int training_samples) {
    std::ofstream output(path);
    require_stream(output, path);
    output << std::setprecision(10);
    output << "{\n";
    output << "  \"training_samples\": " << training_samples << ",\n";
    output << "  \"legacy\": {\n";
    output << "    \"episodes\": " << legacy.episodes << ",\n";
    output << "    \"success_rate\": " << legacy.success_rate << ",\n";
    output << "    \"irreversible_event_rate\": "
           << legacy.irreversible_event_rate << ",\n";
    output << "    \"shell_trigger_rate\": "
           << legacy.shell_trigger_rate << ",\n";
    output << "    \"boundary_expansion_rate\": "
           << legacy.boundary_expansion_rate << ",\n";
    output << "    \"mean_completion_time\": "
           << legacy.mean_completion_time << ",\n";
    output << "    \"mean_control_latency_us\": "
           << legacy.mean_control_latency_us << ",\n";
    output << "    \"p95_control_latency_us\": "
           << legacy.p95_control_latency_us << "\n";
    output << "  },\n";
    output << "  \"reversible\": {\n";
    output << "    \"episodes\": " << reversible.episodes << ",\n";
    output << "    \"success_rate\": " << reversible.success_rate
           << ",\n";
    output << "    \"irreversible_event_rate\": "
           << reversible.irreversible_event_rate << ",\n";
    output << "    \"shell_trigger_rate\": "
           << reversible.shell_trigger_rate << ",\n";
    output << "    \"boundary_expansion_rate\": "
           << reversible.boundary_expansion_rate << ",\n";
    output << "    \"exploratory_interactions\": "
           << reversible.exploratory_interactions << ",\n";
    output << "    \"boundary_expansions\": "
           << reversible.boundary_expansions << ",\n";
    output << "    \"mean_completion_time\": "
           << reversible.mean_completion_time << ",\n";
    output << "    \"mean_rescue_probability\": "
           << reversible.mean_rescue_probability << ",\n";
    output << "    \"mean_epistemic_uncertainty\": "
           << reversible.mean_epistemic_uncertainty << ",\n";
    output << "    \"mean_control_latency_us\": "
           << reversible.mean_control_latency_us << ",\n";
    output << "    \"p95_control_latency_us\": "
           << reversible.p95_control_latency_us << "\n";
    output << "  }\n";
    output << "}\n";
}

void write_comparison_svg(const std::string& path,
                          const AggregateMetrics& legacy,
                          const AggregateMetrics& reversible) {
    std::ofstream output(path);
    require_stream(output, path);
    const double safe_legacy = 1.0 - legacy.irreversible_event_rate;
    const double safe_reversible =
        1.0 - reversible.irreversible_event_rate;
    auto bar = [](double value, double x, const char* color) {
        const double height = 210.0 * clamp01(value);
        return std::string("<rect x=\"") + std::to_string(x) +
               "\" y=\"" + std::to_string(320.0 - height) +
               "\" width=\"78\" height=\"" + std::to_string(height) +
               "\" rx=\"8\" fill=\"" + color + "\"/>";
    };

    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"980\" "
              "height=\"590\" viewBox=\"0 0 980 590\">\n";
    output << "<rect width=\"980\" height=\"590\" fill=\"#0d1117\"/>\n";
    output << "<text x=\"52\" y=\"66\" fill=\"#f0f6fc\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"30\" "
              "font-weight=\"700\">Recoverability-Constructed WBC</text>\n";
    output << "<text x=\"52\" y=\"94\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"15\">"
              "Paired warehouse episodes — construction before learning"
              "</text>\n";
    output << "<line x1=\"52\" y1=\"320\" x2=\"928\" y2=\"320\" "
              "stroke=\"#30363d\"/>\n";
    output << bar(safe_legacy, 120, "#f0883e");
    output << bar(safe_reversible, 206, "#3fb950");
    output << bar(legacy.success_rate, 440, "#f0883e");
    output << bar(reversible.success_rate, 526, "#3fb950");
    output << "<text x=\"120\" y=\"346\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"13\">"
              "legacy</text>\n";
    output << "<text x=\"206\" y=\"346\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"13\">"
              "reversible</text>\n";
    output << "<text x=\"440\" y=\"346\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"13\">"
              "legacy</text>\n";
    output << "<text x=\"526\" y=\"346\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"13\">"
              "reversible</text>\n";
    output << "<text x=\"114\" y=\"390\" fill=\"#c9d1d9\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"14\">"
              "safe episodes</text>\n";
    output << "<text x=\"432\" y=\"390\" fill=\"#c9d1d9\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"14\">"
              "task success</text>\n";
    output << "<text x=\"52\" y=\"458\" fill=\"#3fb950\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"15\">"
              "Boundary expansion / interaction: "
           << std::fixed << std::setprecision(4)
           << reversible.boundary_expansion_rate << "</text>\n";
    output << "<text x=\"52\" y=\"486\" fill=\"#58a6ff\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"15\">"
              "Exploratory interactions: "
           << reversible.exploratory_interactions << "</text>\n";
    output << "<text x=\"52\" y=\"514\" fill=\"#58a6ff\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"15\">"
              "p95 control latency: "
           << std::fixed << std::setprecision(2)
           << reversible.p95_control_latency_us << " us</text>\n";
    output << "<text x=\"52\" y=\"552\" fill=\"#8b949e\" "
              "font-family=\"Avenir,Helvetica,sans-serif\" font-size=\"13\">"
           << legacy.episodes << " paired episodes; execution: "
           << legacy.total_steps << " legacy control steps, "
           << reversible.total_steps
           << " reversible control steps</text>\n";
    output << "</svg>\n";
}

void write_markdown_report(const std::string& path,
                           const AggregateMetrics& legacy,
                           const AggregateMetrics& reversible,
                           int training_samples) {
    std::ofstream output(path);
    require_stream(output, path);
    output << "# Warehouse Reversibility Experiment\n\n";
    output << "The benchmark runs the same route, seeds, payload masses, "
              "friction values, and disturbance phases in both arms.\n\n";
    output << "| Outcome | Legacy command | Constructed shell + learned "
              "expansion |\n";
    output << "|---|---:|---:|\n";
    output << "| Task success | " << percent(legacy.success_rate)
           << " | " << percent(reversible.success_rate) << " |\n";
    output << "| Irreversible event rate | "
           << percent(legacy.irreversible_event_rate) << " | "
           << percent(reversible.irreversible_event_rate) << " |\n";
    output << "| Shell trigger rate | "
           << percent(legacy.shell_trigger_rate) << " | "
           << percent(reversible.shell_trigger_rate) << " |\n";
    output << "| Boundary expansion / interaction | "
           << std::fixed << std::setprecision(4)
           << legacy.boundary_expansion_rate << " | "
           << reversible.boundary_expansion_rate << " |\n";
    output << "| Mean completion time | "
           << std::fixed << std::setprecision(2)
           << legacy.mean_completion_time << " s | "
           << reversible.mean_completion_time << " s |\n";
    output << "| p95 control latency | "
           << std::fixed << std::setprecision(2)
           << legacy.p95_control_latency_us << " us | "
           << reversible.p95_control_latency_us << " us |\n\n";
    output << "Training trials: " << training_samples << ". Exploratory "
              "interactions in the reversible arm: "
           << reversible.exploratory_interactions
           << ". New boundary certificates: "
           << reversible.boundary_expansions << ".\n\n";
    output << "![comparison](comparison.svg)\n";
}

}  // namespace rwb

