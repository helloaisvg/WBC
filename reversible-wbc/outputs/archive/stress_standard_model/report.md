# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 12.50% | 100.00% |
| Irreversible event rate | 87.50% | 0.00% |
| Shell trigger rate | 0.00% | 3.02% |
| Boundary expansion / interaction | 0.0000 | 0.9798 |
| Mean completion time | 21.98 s | 30.25 s |
| p95 control latency | 0.29 us | 0.79 us |

Training trials: 32000. Exploratory interactions in the reversible arm: 1039. New boundary certificates: 1018.

![comparison](comparison.svg)
