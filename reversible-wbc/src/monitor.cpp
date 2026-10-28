#include "rwb/monitor.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

namespace rwb {
namespace {

using ExpandedFeatures = std::array<double, kExpandedFeatureCount>;

ExpandedFeatures expand_features(const FeatureVector& features) {
    ExpandedFeatures expanded{};
    for (std::size_t index = 0; index < kFeatureCount; ++index) {
        expanded[index] = features[index];
        expanded[kFeatureCount + index] =
            features[index] * features[index];
    }
    constexpr std::array<std::array<std::size_t, 2>, 10> pairs{{
        {0, 2}, {1, 3}, {2, 3}, {3, 4}, {4, 8},
        {6, 7}, {8, 9}, {9, 10}, {11, 12}, {12, 13},
    }};
    constexpr std::size_t pair_offset = 2 * kFeatureCount;
    for (std::size_t index = 0; index < pairs.size(); ++index) {
        expanded[pair_offset + index] =
            features[pairs[index][0]] * features[pairs[index][1]];
    }
    constexpr std::size_t hinge_offset =
        pair_offset + pairs.size();
    expanded[hinge_offset] = std::max(0.0, features[8] - 0.54);
    expanded[hinge_offset + 1] = std::max(0.0, features[9] - 0.48);
    expanded[hinge_offset + 2] = std::max(0.0, features[2] - 0.78);
    expanded[hinge_offset + 3] = std::max(0.0, features[6] - 0.50);
    expanded[hinge_offset + 4] = features[9] * features[10];
    expanded[hinge_offset + 5] = features[8] * features[8] * features[9];
    return expanded;
}

double dot_features(
    const std::array<double, kExpandedFeatureCount + 1>& parameters,
    const ExpandedFeatures& features) {
    double result = parameters[kExpandedFeatureCount];
    for (std::size_t index = 0; index < kExpandedFeatureCount; ++index) {
        result += parameters[index] * features[index];
    }
    return result;
}

}  // namespace

FeatureVector extract_features(const RobotState& state, const Action& action,
                               const ShellDecision& shell,
                               const SafetyShell& model_shell) {
    const ShellLimits& limits = model_shell.limits();
    FeatureVector features{};
    const double linear_accel =
        std::abs((action.linear_velocity - state.linear_velocity) /
                 limits.control_dt);
    const double lateral_accel =
        std::abs(state.linear_velocity * action.angular_velocity);
    const double joint_speed =
        std::max(std::abs(state.joint_velocity_1),
                 std::abs(state.joint_velocity_2));
    const double joint_target_error = std::max(
        std::abs(action.joint_1_target - state.joint_1) /
            std::max(limits.max_joint_1 - limits.min_joint_1, 1e-9),
        std::abs(action.joint_2_target - state.joint_2) /
            std::max(limits.max_joint_2 - limits.min_joint_2, 1e-9));
    const double angular_accel =
        std::abs((action.angular_velocity - state.angular_velocity) /
                 limits.control_dt);

    features[0] =
        clamp01(std::abs(state.linear_velocity) / limits.max_linear_speed);
    features[1] =
        clamp01(std::abs(state.angular_velocity) / limits.max_angular_speed);
    features[2] =
        clamp01(linear_accel / limits.max_linear_accel);
    features[3] =
        clamp01(angular_accel / limits.max_angular_accel);
    features[4] =
        clamp01(lateral_accel / limits.max_lateral_accel);
    features[5] = model_shell.arm_extension(state);
    features[6] = clamp01(joint_target_error);
    features[7] = clamp01(joint_speed / 2.4);
    features[8] = clamp01(std::abs(state.roll) / limits.max_roll);
    features[9] = clamp01(std::abs(state.payload_angle) /
                          limits.payload_fall_angle);
    features[10] = clamp01(std::abs(state.payload_rate) / 2.5);
    features[11] = clamp01((state.payload_mass - 3.0) / 11.0);
    features[12] = clamp01((1.40 - state.friction) / 0.58);
    features[13] =
        1.0 - clamp01((state.obstacle_clearance - limits.base_radius) /
                      1.60);
    features[14] =
        clamp01((action.joint_1_target - limits.min_joint_1) /
                std::max(limits.max_joint_1 - limits.min_joint_1, 1e-9));
    features[15] =
        clamp01((action.joint_2_target - limits.min_joint_2) /
                std::max(limits.max_joint_2 - limits.min_joint_2, 1e-9));
    features[16] =
        clamp01(std::abs(state.external_disturbance) / 6.0);

    if (shell.acceleration_utilization > 1.0) {
        features[2] = 1.0;
    }
    return features;
}

RecoverabilityMonitor::RecoverabilityMonitor(MonitorLimits limits)
    : limits_(limits) {}

void RecoverabilityMonitor::add_training_sample(const FeatureVector& features,
                                                 bool recovered) {
    training_samples_.push_back({features, recovered});
}

void RecoverabilityMonitor::clear_training_samples() {
    training_samples_.clear();
}

void RecoverabilityMonitor::update_feature_statistics() {
    centroid_.fill(0.0);
    spread_.fill(0.0);
    if (training_samples_.empty()) {
        return;
    }
    for (const TrainingSample& sample : training_samples_) {
        for (std::size_t index = 0; index < kFeatureCount; ++index) {
            centroid_[index] += sample.features[index];
        }
    }
    for (double& value : centroid_) {
        value /= static_cast<double>(training_samples_.size());
    }
    for (const TrainingSample& sample : training_samples_) {
        for (std::size_t index = 0; index < kFeatureCount; ++index) {
            spread_[index] +=
                sq(sample.features[index] - centroid_[index]);
        }
    }
    for (double& value : spread_) {
        value = std::sqrt(value /
                          static_cast<double>(training_samples_.size()));
        value = std::max(value, 0.045);
    }
}

MonitorTrainingResult RecoverabilityMonitor::train(std::uint64_t seed,
                                                    int ensemble_size,
                                                    int epochs) {
    MonitorTrainingResult result;
    result.samples = static_cast<int>(training_samples_.size());
    for (const TrainingSample& sample : training_samples_) {
        result.positive += sample.recovered ? 1 : 0;
    }
    result.negative = result.samples - result.positive;
    if (training_samples_.size() < 32 || result.positive == 0 ||
        result.negative == 0) {
        return result;
    }

    update_feature_statistics();
    source_sample_count_ = result.samples;
    ensemble_size = std::max(1, ensemble_size);
    epochs = std::max(20, epochs);
    weights_.clear();
    online_shift_.fill(0.0);

    Rng rng(seed);
    std::vector<std::size_t> shuffled(training_samples_.size());
    std::iota(shuffled.begin(), shuffled.end(), 0U);
    for (std::size_t index = shuffled.size(); index > 1; --index) {
        const std::size_t other = static_cast<std::size_t>(
            rng.uniform_int(0, static_cast<int>(index) - 1));
        std::swap(shuffled[index - 1], shuffled[other]);
    }
    const std::size_t calibration_size =
        std::max<std::size_t>(64U, shuffled.size() / 5U);
    const std::size_t training_size =
        shuffled.size() - calibration_size;
    std::vector<std::size_t> training_indices(
        shuffled.begin(), shuffled.begin() + training_size);
    std::vector<std::size_t> calibration_indices(
        shuffled.begin() + training_size, shuffled.end());
    int training_positive = 0;
    for (std::size_t index : training_indices) {
        training_positive += training_samples_[index].recovered ? 1 : 0;
    }
    const int training_negative =
        static_cast<int>(training_indices.size()) - training_positive;
    if (training_positive < 8 || training_negative < 8) {
        return result;
    }

    const double positive_weight =
        static_cast<double>(training_indices.size()) /
        (2.0 * static_cast<double>(training_positive));
    const double negative_weight =
        static_cast<double>(training_indices.size()) /
        (2.0 * static_cast<double>(training_negative));
    const double initial_bias =
        std::log(static_cast<double>(training_positive) /
                 static_cast<double>(training_negative));

    double accumulated_loss = 0.0;
    double accumulated_confidence = 0.0;
    double accumulated_squared_error = 0.0;

    for (int model_index = 0; model_index < ensemble_size; ++model_index) {
        ParameterVector parameters{};
        parameters[kExpandedFeatureCount] = initial_bias;
        std::vector<std::size_t> bootstrap;
        bootstrap.reserve(training_samples_.size());
        for (std::size_t index = 0; index < training_indices.size(); ++index) {
            bootstrap.push_back(
                static_cast<std::size_t>(rng.uniform_int(
                    0, static_cast<int>(training_indices.size()) - 1)));
        }

        for (int epoch = 0; epoch < epochs; ++epoch) {
            ParameterVector gradient{};
            double loss = 0.0;
            for (std::size_t bootstrap_index : bootstrap) {
                const std::size_t sample_index =
                    training_indices[bootstrap_index];
                const TrainingSample& sample =
                    training_samples_[sample_index];
                const ExpandedFeatures expanded =
                    expand_features(sample.features);
                const double prediction =
                    logistic(dot_features(parameters, expanded));
                const double target = sample.recovered ? 1.0 : 0.0;
                const double class_weight =
                    sample.recovered ? positive_weight : negative_weight;
                const double error =
                    (prediction - target) * class_weight;
                loss -= class_weight *
                        (target * std::log(std::max(prediction, 1e-9)) +
                         (1.0 - target) *
                             std::log(std::max(1.0 - prediction, 1e-9)));
                for (std::size_t feature = 0;
                     feature < kExpandedFeatureCount;
                     ++feature) {
                    gradient[feature] += error * expanded[feature];
                }
                gradient[kExpandedFeatureCount] += error;
            }

            const double inverse_size =
                1.0 / static_cast<double>(bootstrap.size());
            const double learning_rate =
                0.19 / (1.0 + 0.0035 * static_cast<double>(epoch));
            for (std::size_t parameter = 0; parameter < kParameterCount;
                 ++parameter) {
                double gradient_value = gradient[parameter] * inverse_size;
                if (parameter < kExpandedFeatureCount) {
                    gradient_value +=
                        limits_.regularization * parameters[parameter];
                }
                parameters[parameter] -=
                    learning_rate * gradient_value;
            }
            if (epoch == epochs - 1) {
                accumulated_loss += loss * inverse_size;
            }
        }
        weights_.push_back(parameters);
    }

    for (std::size_t sample_index : calibration_indices) {
        const TrainingSample& sample =
            training_samples_[sample_index];
        const double probability =
            predict(sample.features).rescue_probability;
        const double target = sample.recovered ? 1.0 : 0.0;
        accumulated_confidence += std::abs(probability - target);
        accumulated_squared_error += sq(probability - target);
    }
    result.final_loss =
        accumulated_loss / static_cast<double>(ensemble_size);
    result.mean_confidence =
        1.0 - accumulated_confidence /
                  static_cast<double>(training_samples_.size());
    result.brier_score =
        accumulated_squared_error /
        static_cast<double>(training_samples_.size());

    std::vector<std::pair<double, bool>> ranked;
    ranked.reserve(calibration_indices.size());
    int positive_correct = 0;
    int negative_correct = 0;
    int calibration_positive = 0;
    std::vector<double> negative_scores;
    for (std::size_t sample_index : calibration_indices) {
        const TrainingSample& sample =
            training_samples_[sample_index];
        const double probability =
            predict(sample.features).rescue_probability;
        ranked.emplace_back(probability, sample.recovered);
        calibration_positive += sample.recovered ? 1 : 0;
        if (sample.recovered && probability >= 0.5) {
            ++positive_correct;
        }
        if (!sample.recovered && probability < 0.5) {
            ++negative_correct;
        }
        if (!sample.recovered) {
            negative_scores.push_back(probability);
        }
    }
    const int calibration_negative =
        static_cast<int>(calibration_indices.size()) -
        calibration_positive;
    result.positive_accuracy =
        calibration_positive > 0
            ? static_cast<double>(positive_correct) /
                  static_cast<double>(calibration_positive)
            : 0.0;
    result.negative_accuracy =
        calibration_negative > 0
            ? static_cast<double>(negative_correct) /
                  static_cast<double>(calibration_negative)
            : 0.0;

    std::sort(ranked.begin(), ranked.end(),
              [](const auto& left, const auto& right) {
                  return left.first > right.first;
              });
    double positive_rank_sum = 0.0;
    std::size_t position = 0;
    while (position < ranked.size()) {
        std::size_t end = position + 1;
        while (end < ranked.size() &&
               std::abs(ranked[end].first - ranked[position].first) <
                   1e-12) {
            ++end;
        }
        const double average_rank =
            0.5 * (static_cast<double>(position + 1) +
                   static_cast<double>(end));
        for (std::size_t index = position; index < end; ++index) {
            if (ranked[index].second) {
                positive_rank_sum += average_rank;
            }
        }
        position = end;
    }
    result.auc =
        1.0 -
        (positive_rank_sum -
         static_cast<double>(calibration_positive) *
             static_cast<double>(calibration_positive + 1) * 0.5) /
            (static_cast<double>(calibration_positive) *
             static_cast<double>(calibration_negative));

    std::sort(negative_scores.begin(), negative_scores.end());
    const auto quantile = [&negative_scores](double value) {
        const std::size_t index = std::min(
            negative_scores.size() - 1,
            static_cast<std::size_t>(
                std::floor(value *
                           static_cast<double>(
                               negative_scores.size() - 1))));
        return negative_scores[index];
    };
    strict_threshold_ = clamp(quantile(0.90), 0.45, 0.97);
    exploration_threshold_ =
        clamp(std::min(strict_threshold_ - 0.02, quantile(0.75)),
              0.40, strict_threshold_);
    return result;
}

double RecoverabilityMonitor::online_probability(
    const ParameterVector& parameters, const FeatureVector& features) const {
    const ExpandedFeatures expanded = expand_features(features);
    double score = dot_features(parameters, expanded);
    score += online_shift_[kFeatureCount];
    for (std::size_t index = 0; index < kFeatureCount; ++index) {
        score += online_shift_[index] * features[index];
    }
    return logistic(score);
}

MonitorPrediction RecoverabilityMonitor::predict(
    const FeatureVector& features) const {
    MonitorPrediction prediction;
    if (weights_.empty()) {
        return prediction;
    }

    double mean = 0.0;
    double second_moment = 0.0;
    for (const ParameterVector& parameters : weights_) {
        const double probability = online_probability(parameters, features);
        mean += probability;
        second_moment += probability * probability;
    }
    const double inverse_count =
        1.0 / static_cast<double>(weights_.size());
    mean *= inverse_count;
    second_moment *= inverse_count;
    const double ensemble_std =
        std::sqrt(std::max(0.0, second_moment - mean * mean));

    double normalized_distance = 0.0;
    for (std::size_t index = 0; index < kFeatureCount; ++index) {
        normalized_distance +=
            sq((features[index] - centroid_[index]) /
               std::max(spread_[index], 1e-6));
    }
    normalized_distance /= static_cast<double>(kFeatureCount);
    const double coverage =
        std::exp(-0.5 * std::min(normalized_distance, 12.0));
    const double uncertainty = clamp01(
        ensemble_std + 0.30 * (1.0 - coverage));

    prediction.rescue_probability = mean;
    prediction.epistemic_uncertainty = uncertainty;
    prediction.coverage = coverage;
    prediction.strict_threshold = strict_threshold_;
    prediction.exploration_threshold = exploration_threshold_;
    prediction.components = {
        mean, ensemble_std, coverage, uncertainty, normalized_distance};
    return prediction;
}

void RecoverabilityMonitor::adapt_online(const FeatureVector& features,
                                         bool recovered) {
    if (weights_.empty()) {
        return;
    }
    double mean = 0.0;
    for (const ParameterVector& parameters : weights_) {
        mean += online_probability(parameters, features);
    }
    mean /= static_cast<double>(weights_.size());
    const double target = recovered ? 1.0 : 0.0;
    double error = target - mean;
    error = clamp(error, -0.85, 0.85);
    const double learning_rate =
        limits_.learning_rate * limits_.online_update_fraction;

    const std::array<std::size_t, 8> update_indices{
        2U, 3U, 4U, 6U, 8U, 9U, 13U, kFeatureCount};
    for (std::size_t index : update_indices) {
        const double feature =
            index < kFeatureCount ? features[index] : 1.0;
        online_shift_[index] += learning_rate * error * feature;
        online_shift_[index] =
            clamp(online_shift_[index], -limits_.max_online_shift,
                  limits_.max_online_shift);
    }
}

bool RecoverabilityMonitor::save(const std::string& path,
                                 std::string& error) const {
    if (weights_.empty()) {
        error = "monitor has no trained ensemble";
        return false;
    }
    std::ofstream output(path);
    if (!output) {
        error = "cannot open model output: " + path;
        return false;
    }
    output << "RWBMODEL 1\n";
    output << "SAMPLES " << training_samples_.size() << '\n';
    output << "FEATURES " << kFeatureCount << '\n';
    output << "ENSEMBLE " << weights_.size() << '\n';
    output << "THRESHOLDS " << std::setprecision(17)
           << strict_threshold_ << ' ' << exploration_threshold_ << '\n';
    output << "CENTROID";
    for (double value : centroid_) {
        output << ' ' << std::setprecision(17) << value;
    }
    output << "\nSPREAD";
    for (double value : spread_) {
        output << ' ' << std::setprecision(17) << value;
    }
    output << "\nONLINE";
    for (double value : online_shift_) {
        output << ' ' << std::setprecision(17) << value;
    }
    output << '\n';
    for (const ParameterVector& parameters : weights_) {
        output << "WEIGHTS";
        for (double value : parameters) {
            output << ' ' << std::setprecision(17) << value;
        }
        output << '\n';
    }
    return true;
}

bool RecoverabilityMonitor::load(const std::string& path,
                                 std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "cannot open model: " + path;
        return false;
    }
    std::string token;
    int version = 0;
    std::size_t sample_count = 0;
    std::size_t feature_count = 0;
    std::size_t ensemble_count = 0;
    input >> token >> version;
    if (token != "RWBMODEL" || version != 1) {
        error = "unsupported model format";
        return false;
    }
    input >> token >> sample_count >> token >> feature_count >> token >>
        ensemble_count;
    if (token != "ENSEMBLE" || feature_count != kFeatureCount ||
        ensemble_count == 0) {
        error = "invalid model header";
        return false;
    }

    input >> token;
    if (token != "THRESHOLDS") {
        error = "missing calibrated thresholds";
        return false;
    }
    input >> strict_threshold_ >> exploration_threshold_;
    input >> token;
    if (token != "CENTROID") {
        error = "missing centroid";
        return false;
    }
    for (double& value : centroid_) {
        input >> value;
    }
    input >> token;
    if (token != "SPREAD") {
        error = "missing spread";
        return false;
    }
    for (double& value : spread_) {
        input >> value;
    }
    input >> token;
    if (token != "ONLINE") {
        error = "missing online state";
        return false;
    }
    for (double& value : online_shift_) {
        input >> value;
    }

    weights_.clear();
    for (std::size_t model = 0; model < ensemble_count; ++model) {
        input >> token;
        if (token != "WEIGHTS") {
            error = "missing model weights";
            return false;
        }
        ParameterVector parameters{};
        for (double& value : parameters) {
            input >> value;
        }
        weights_.push_back(parameters);
    }
    if (!input) {
        error = "model ended unexpectedly";
        weights_.clear();
        return false;
    }
    training_samples_.clear();
    training_samples_.reserve(sample_count);
    source_sample_count_ = static_cast<int>(sample_count);
    return true;
}

}  // namespace rwb

