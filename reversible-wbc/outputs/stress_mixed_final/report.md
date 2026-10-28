# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 12.50% | 100.00% |
| Irreversible event rate | 87.50% | 0.00% |
| Shell trigger rate | 0.00% | 1.57% |
| Boundary expansion / interaction | 0.0000 | 0.9805 |
| Mean completion time | 21.98 s | 30.54 s |
| p95 control latency | 0.42 us | 1.25 us |

Training trials: 34028. Exploratory interactions in the reversible arm: 821. New boundary certificates: 805.

![comparison](comparison.svg)
