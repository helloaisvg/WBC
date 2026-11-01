# REMANI-R Ablation Experiments

## Protocol

- Environment: Ubuntu 20.04, ROS Noetic, x86_64 container on Apple Silicon.
- Scenes: dense cuboids (`exp0`) and bridge (`exp1`).
- Map seeds: `30`, `31`, `32`.
- Modes:
  - `remani`: original collision-based objective.
  - `braking`: base stopping-distance envelope.
  - `whole_body`: stopping envelope with arm-extension coupling.
  - `full`: stopping envelope, arm-extension coupling, recovery-time
    manifold, runtime monitoring, and temporal safety repair.
- Each run executes two planned trajectories.
- Total: 24 runs and 48 planned trajectories.

Raw data: `research/results/tables/recovery_ablation.csv`.

## TopAY External Baseline

TopAY was compiled in the existing ROS container and run on 100 fixed-random
`8 m x 8 m` cuboid cases with 20 ground and 30 floating obstacles.

- TopAY MOMA success: `52/100`
- Mean planning time on successful cases: `122.17 ms`
- Mean trajectory duration on successful cases: `8.90722 s`

The measured value is documented in
[TOPAY_COMPARISON.md](../external/TOPAY_COMPARISON.md). Raw output is
`research/archive/raw_logs/topay_benchmark100.log`.

## GCDF External Baseline

GCDF Table II reports REMANI, TopAY and GCDF under one Kinova Gen3 protocol:
[GCDF_COMPARISON.md](../external/GCDF_COMPARISON.md).

At the published 120-obstacle tier:

- REMANI: `49.6%`, `0.3292 s`
- TopAY: `51.6%`, `0.2727 s`
- GCDF: `86.8%`, `0.2871 s`
- GCDF with TopAY initialization: `91.6%`, `0.1957 s`

## Results

| Mode | Scene | Runs | Trajectories | Success | Mean wall | P95 wall | Worst margin | Mean slack | Worst slack |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| REMANI | `exp0` | 3 | 6 | 100% | 382.3 ms | 437.7 ms | -0.2004 m | -0.0214 s | -0.9854 s |
| Braking | `exp0` | 3 | 6 | 100% | 398.5 ms | 465.7 ms | -0.0577 m | -0.1462 s | -0.6833 s |
| Whole-body | `exp0` | 3 | 6 | 100% | 435.1 ms | 477.0 ms | -0.0518 m | -0.0587 s | -0.5051 s |
| Full | `exp0` | 3 | 6 | 100% | 421.9 ms | 474.6 ms | -0.0577 m | +0.2133 s | -0.0235 s |
| REMANI | `exp1` | 3 | 6 | 100% | 524.7 ms | 591.3 ms | -0.1911 m | -0.4509 s | -0.5215 s |
| Braking | `exp1` | 3 | 6 | 100% | 462.4 ms | 522.1 ms | -0.0383 m | -0.6376 s | -0.9159 s |
| Whole-body | `exp1` | 3 | 6 | 100% | 478.9 ms | 561.3 ms | -0.0309 m | -0.4380 s | -0.5343 s |
| Full | `exp1` | 3 | 6 | 100% | 527.0 ms | 604.1 ms | -0.0490 m | -0.0328 s | -0.0520 s |

No run produced a collision rejection or emergency stop.

## Findings

- The original REMANI reached the goal in all runs but had braking deficits
  up to `0.2004 m` and recovery-time deficits up to `0.9854 s`.
- The base braking envelope reduced the worst braking deficit to
  `0.0577 m` in `exp0` and `0.0383 m` in `exp1`.
- The full recoverability manifold reduced the worst recovery-time deficit to
  `0.0235 s` in `exp0` and `0.0520 s` in `exp1`.
- Full `exp0` mean planning time is `39.6 ms` above REMANI and P95 is
  `36.9 ms` above REMANI.
- Full `exp1` mean planning time is `2.3 ms` above REMANI and P95 is
  `12.8 ms` above REMANI.
- The added whole-body recovery guarantee has a planning-latency delta of
  `0.4%` to `10.4%` on these scenes.

## Final Five-Seed Regression

Final configuration:

- static: original `80 ms` optimization budget and iteration ceiling;
- dynamic: `60 ms` optimization budget, trajectory-arrival-time prediction,
  localized temporal repair, and safety-audited piecewise time compression;
- all final audits still run at `0.01 s`;
- dynamic front-end collision checks remain at `12` samples.

Static results use seeds `30` through `34` and two planned trajectories per
seed:

| Method | Scene | Runs | Two-plan success | Mean wall | P95 wall | Metric violations |
|---|---|---:|---:|---:|---:|---:|
| REMANI-R final | `exp0` | 5 | 5/5 | 243.7 ms | 286.4 ms | 0 |
| REMANI-R final | `exp1_random` | 5 | 5/5 | 235.3 ms | 293.7 ms | 0 |

Compared with the previous REMANI-R static measurements:

- `exp0`: mean `261.8 -> 243.7 ms`, P95 `316.1 -> 286.4 ms`;
- `exp1_random`: mean `245.1 -> 235.3 ms`, P95 `361.6 -> 293.7 ms`.

Dynamic moving-obstacle mission, seeds `30` through `34`, two crossing
spheres, 20 ground and 30 floating static obstacles:

| Method | Seeds | Mission success | Mean wall | P95 wall | Mean trajectory | Safety violations | Replans | Emergency stops |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| REMANI-R final | 5 | 5/5 | 276.8 ms | 357.7 ms | 18.72 s | 0 | 0 | 0 |

Three-seed comparison:

| Method | Runs | Mean wall | P95 wall | Mean trajectory | Safety violations | Replans | Emergency stops |
|---|---:|---:|---:|---:|---:|---:|---:|
| REMANI | 3 | 214.0 ms | 289.5 ms | 16.65 s | 9 | 4 | 1 |
| REMANI-R before final iteration | 3 | 217.0 ms | 389.2 ms | 20.64 s | 0 | 2 | 0 |
| REMANI-R final | 3 | 258.4 ms | 305.5 ms | 19.35 s | 0 | 0 | 0 |

Raw final data:

- `research/results/canonical/static_final_buffer.csv`
- `research/results/canonical/static_final_buffer_exp1r_tail.csv`
- `research/results/canonical/dynamic_final5_buffer.csv`

Rejected tuning variants:

- weighted A* at `lambda_heu = 1.1` and `1.3` worsened path duration and tail
  latency;
- dynamic front-end checking at 10 samples produced 59.5 s and 63.5 s
  trajectories in the five-seed regression;
- stronger temporal anchors and doubled obstacle/recoverability weights were
  less stable than the selected configuration.

## Technical Positioning

REMANI-R combines five capabilities in one whole-body planning loop:

1. nonholonomic whole-body mobile-manipulator trajectory optimization;
2. configuration-dependent base stopping distance;
3. joint-level recovery-time constraints to a low-inertia posture;
4. runtime recoverability monitoring; and
5. exact polynomial time-stretch repair.
