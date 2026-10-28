# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 10.00% | 100.00% |
| Irreversible event rate | 90.00% | 0.00% |
| Shell trigger rate | 0.00% | 23.50% |
| Boundary expansion / interaction | 0.0000 | 0.9963 |
| Mean completion time | 21.98 s | 28.97 s |
| p95 control latency | 0.50 us | 0.62 us |

Training trials: 32000. Exploratory interactions in the reversible arm: 269. New boundary certificates: 268.

![comparison](comparison.svg)
