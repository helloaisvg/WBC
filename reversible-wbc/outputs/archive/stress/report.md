# Warehouse Reversibility Experiment

The benchmark runs the same route, seeds, payload masses, friction values, and disturbance phases in both arms.

| Outcome | Legacy command | Constructed shell + learned expansion |
|---|---:|---:|
| Task success | 20.00% | 5.00% |
| Irreversible event rate | 80.00% | 95.00% |
| Shell trigger rate | 0.00% | 36.57% |
| Boundary expansion / interaction | 0.0000 | 0.9205 |
| Mean completion time | 21.98 s | 32.81 s |
| p95 control latency | 0.21 us | 0.54 us |

Training trials: 32000. Exploratory interactions in the reversible arm: 365. New boundary certificates: 336.

![comparison](comparison.svg)
