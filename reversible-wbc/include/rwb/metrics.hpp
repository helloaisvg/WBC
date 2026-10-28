#pragma once

#include <string>
#include <vector>

#include "rwb/types.hpp"

namespace rwb {

struct AggregateMetrics {
    int episodes = 0;
    int successful_episodes = 0;
    int irreversible_episodes = 0;
    long long total_steps = 0;
    long long shell_triggers = 0;
    long long gate_triggers = 0;
    long long exploratory_interactions = 0;
    long long boundary_expansions = 0;

    double success_rate = 0.0;
    double irreversible_event_rate = 0.0;
    double shell_trigger_rate = 0.0;
    double boundary_expansion_rate = 0.0;
    double mean_completion_time = 0.0;
    double mean_final_goal_distance = 0.0;
    double mean_rescue_probability = 0.0;
    double mean_epistemic_uncertainty = 0.0;
    double mean_control_latency_us = 0.0;
    double p95_control_latency_us = 0.0;
};

AggregateMetrics aggregate_metrics(const std::vector<EpisodeResult>& episodes);

void write_episode_csv(const std::string& path,
                       const std::vector<EpisodeResult>& episodes);
void write_summary_json(const std::string& path,
                        const AggregateMetrics& legacy,
                        const AggregateMetrics& reversible,
                        int training_samples);
void write_comparison_svg(const std::string& path,
                          const AggregateMetrics& legacy,
                          const AggregateMetrics& reversible);
void write_markdown_report(const std::string& path,
                           const AggregateMetrics& legacy,
                           const AggregateMetrics& reversible,
                           int training_samples);

}  // namespace rwb

