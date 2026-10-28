#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "rwb/safety_shell.hpp"

namespace rwb {

constexpr std::size_t kFeatureCount = 17;
constexpr std::size_t kExpandedFeatureCount = 50;
using FeatureVector = std::array<double, kFeatureCount>;

FeatureVector extract_features(const RobotState& state, const Action& action,
                               const ShellDecision& shell,
                               const SafetyShell& model_shell);

struct TrainingSample {
    FeatureVector features;
    bool recovered = false;
};

struct MonitorTrainingResult {
    int samples = 0;
    int positive = 0;
    int negative = 0;
    double final_loss = 0.0;
    double mean_confidence = 0.0;
    double brier_score = 1.0;
    double auc = 0.5;
    double positive_accuracy = 0.0;
    double negative_accuracy = 0.0;
};

class RecoverabilityMonitor {
public:
    explicit RecoverabilityMonitor(MonitorLimits limits);

    void add_training_sample(const FeatureVector& features, bool recovered);
    void clear_training_samples();

    MonitorTrainingResult train(std::uint64_t seed, int ensemble_size = 5,
                                int epochs = 260);
    MonitorPrediction predict(const FeatureVector& features) const;
    void adapt_online(const FeatureVector& features, bool recovered);

    bool save(const std::string& path, std::string& error) const;
    bool load(const std::string& path, std::string& error);

    bool trained() const { return !weights_.empty(); }
    int training_sample_count() const { return source_sample_count_; }
    int ensemble_size() const { return static_cast<int>(weights_.size()); }
    const MonitorLimits& limits() const { return limits_; }
    double strict_threshold() const { return strict_threshold_; }
    double exploration_threshold() const {
        return exploration_threshold_;
    }

private:
    static constexpr std::size_t kParameterCount =
        kExpandedFeatureCount + 1;
    using ParameterVector = std::array<double, kParameterCount>;
    using OnlineVector = std::array<double, kFeatureCount + 1>;

    MonitorLimits limits_;
    int source_sample_count_ = 0;
    std::vector<TrainingSample> training_samples_;
    FeatureVector centroid_{};
    FeatureVector spread_{};
    double strict_threshold_ = 1.0;
    double exploration_threshold_ = 1.0;
    std::vector<ParameterVector> weights_;
    OnlineVector online_shift_{};

    double online_probability(const ParameterVector& parameters,
                              const FeatureVector& features) const;
    void update_feature_statistics();
};

}  // namespace rwb

