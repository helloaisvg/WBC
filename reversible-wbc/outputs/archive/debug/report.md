# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 100.00% | 100.00% |
| Irreversible event rate | 0.00% | 0.00% |
| Shell trigger rate | 0.00% | 25.11% |
| Boundary expansion / interaction | 0.0000 | 1.0000 |
| Mean completion time | 21.98 s | 26.60 s |
| p95 control latency | 0.46 us | 0.46 us |

Training trials: 32000. Exploratory interactions in the reversible arm: 15. New boundary certificates: 15.

![comparison](comparison.svg)
