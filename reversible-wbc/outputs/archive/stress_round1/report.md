# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 20.00% | 100.00% |
| Irreversible event rate | 80.00% | 0.00% |
| Shell trigger rate | 0.00% | 3.80% |
| Boundary expansion / interaction | 0.0000 | 0.9719 |
| Mean completion time | 21.98 s | 30.10 s |
| p95 control latency | 0.29 us | 0.62 us |

Training trials: 32000. Exploratory interactions in the reversible arm: 1066. New boundary certificates: 1036.

![comparison](comparison.svg)
