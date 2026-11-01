# Fair Dynamic Benchmark

## Shared setup

- Map: seed 30, 8 x 8 m, shared PCD.
- Robot: identical mobile manipulator geometry.
- Start: `[-3.0, 0.0]`.
- Goals: `[3.0, 1.5]`, then `[-0.5, -3.0]`.
- Arm posture: `[0, -70, 110, 0, -110, 0]` degrees at the start and both goals.
- Recording window: 60 seconds.

## Shared dynamic input

All planners receive the same tracked sphere:

- Radius: 0.45 m.
- Center height: 0.62 m.
- Motion: straight line at `x = 0.2`, from `y = 1.8` down at 0.2 m/s.
- Track fields: position, radius, velocity.

The original REMANI and TopAY use the current obstacle position only. REMANI-R uses position and velocity to evaluate future collision time.

## Dynamic adapters

- Original REMANI: current-track distance added to the original ESDF collision query.
- REMANI-R: native time-aware dynamic obstacle pipeline.
- TopAY: current-track distance added to the original grid-map collision query.

## Recorded results

| Planner | Dynamic model | Goals | Emergency stops | Result |
|---|---|---:|---:|---|
| Original REMANI | Current position | 2/2 | 1 | Reactive replan and recovery |
| REMANI-R | Current position + velocity prediction | 2/2 | 0 | Predictive avoidance |
| TopAY | Current occupied map | 0/2 | 0 | No feasible initial topological path |

## Video files

- `fair_original_dynamic.mp4`
- `fair_remani_r_dynamic.mp4`
- `fair_topay_dynamic.mp4`
