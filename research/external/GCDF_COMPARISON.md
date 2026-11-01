# GCDF Comparison

## Identification

GCDF is the method in:

> Fast and Safe Trajectory Optimization for Mobile Manipulators With Neural
> Configuration Space Distance Field

- Paper: https://arxiv.org/abs/2601.18548
- Code: https://github.com/YulinLi0/GCDF
- Robot in the public implementation: Kinova Gen3 mobile manipulator

## Published Same-Protocol Comparison

GCDF Table II already compares GCDF, REMANI, and TopAY under the same protocol:

- `14 m × 14 m` map;
- Kinova Gen3 mobile manipulator;
- 50 feasible goals on each of 5 random maps per density;
- 250 planning trials for each obstacle tier;
- identical success and runtime definitions.

Times are seconds per trial. `GCDF + TopAY init` uses TopAY's front-end
trajectory only as the initial guess for the GCDF solver.

| Obstacles | Method | Success | Runtime |
|---:|---|---:|---:|
| 80 | REMANI | 86.8% | 0.2858 s |
| 80 | TopAY | 88.4% | 0.1393 s |
| 80 | GCDF | 92.4% | 0.1732 s |
| 80 | GCDF + TopAY init | **98.0%** | **0.1341 s** |
| 100 | REMANI | 61.5% | 0.3077 s |
| 100 | TopAY | 67.2% | 0.1776 s |
| 100 | GCDF | 88.4% | 0.2446 s |
| 100 | GCDF + TopAY init | **92.5%** | **0.1641 s** |
| 120 | REMANI | 49.6% | 0.3292 s |
| 120 | TopAY | 51.6% | 0.2727 s |
| 120 | GCDF | 86.8% | 0.2871 s |
| 120 | GCDF + TopAY init | **91.6%** | **0.1957 s** |

## What This Means

- GCDF is a substantially stronger collision-reasoning baseline than the
  current REMANI and TopAY implementations at high obstacle density.
- TopAY is generally faster, but its front-end success collapses from 88.4%
  to 51.6% as density increases.
- GCDF plus TopAY initialization is the strongest published combination in
  this table: 91.6% success at 120 obstacles and 0.1957 s average runtime.
