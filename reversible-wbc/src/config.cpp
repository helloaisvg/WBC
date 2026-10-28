#include "rwb/config.hpp"

namespace rwb {

ScenarioConfig::ScenarioConfig()
    : obstacles{
          {1.95, 0.70, 2.45, 2.80},
          {4.30, -2.80, 4.80, -1.85},
          {6.55, 0.40, 7.05, 2.80},
          {8.70, -2.80, 9.20, -0.80},
      },
      waypoints{
          {2.90, -1.55},
          {3.55, -0.35},
          {5.90, -0.72},
          {7.45, -0.72},
          {8.10, 1.35},
          {10.55, 1.35},
      } {}

ScenarioConfig ScenarioConfig::warehouse() {
    return ScenarioConfig{};
}

}  // namespace rwb

