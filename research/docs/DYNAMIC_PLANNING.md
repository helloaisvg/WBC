# Time-Aware Dynamic Obstacle Planning

## Problem

The original global-map path used a static point cloud and returned after the
first map callback. It handled motion through replacement point clouds, each
exposing the obstacle's current location. A trajectory reaching the same
region several seconds later needs the obstacle state at arrival time.

REMANI-R now represents dynamic obstacles in the query layer used by search,
optimization, recovery auditing, and FSM execution.

## Representation

Each obstacle is a sphere with:

```text
center(t) = center_0 + velocity * (t - t_0)
signed_distance(p, t) = ||p - center(t)|| - radius
```

The `world_time` argument is the time at which the robot reaches trajectory
sample `p`. Planner wall-clock evaluation time is separate.

The static ESDF remains unchanged. Every distance query returns the minimum
of:

1. trilinearly interpolated static ESDF distance;
2. analytic signed distance to every active dynamic sphere at `world_time`.

For the dynamic term, the spatial gradient is also analytic:

```text
gradient = (p - center(t)) / ||p - center(t)||
```

No additional grid update or ESDF recomputation is required when an obstacle
moves.

## Map Readiness

The FSM accepts a preset mission after both the static cloud and the initial
ESDF have been produced. This readiness gate prevents a random-map run from
planning against the initialized `10000 m` sentinel distance before the first
map update.

The readiness condition is:

```text
static cloud received && first ESDF update completed
```

The trigger remains queued and the FSM activates the mission event-driven
when the condition becomes true.

## Integration

- Hybrid A* and manipulator sampling use the obstacle set at the current
  simulation time.
- L-BFGS evaluates each sample at `trajectory_start_time + sample_time`.
- The 0.01 s final safety scan checks collision and recoverability at the
  corresponding absolute time.
- The FSM runtime timer checks the current obstacle predictions at every
  future trajectory time it audits.
- Temporal repair locates the first failing sample, lengthens only the
  polynomial suffix, and regenerates coefficients with fixed endpoint states.
  This preserves polynomial continuity while avoiding a uniform stretch of
  the entire route.
- After optimization, safe time allocation is compacted piece by piece toward
  the A* seed. Candidate checks use `0.05 s` sampling during search; the final
  selected path is always re-audited at `0.01 s`.

The implementation touches:

- `plan_env/include/plan_env/grid_map.h`
- `plan_env/src/grid_map.cpp`
- `mm_config/include/mm_config/mm_config.hpp`
- `mm_config/src/mm_config.cpp`
- `traj_opt/include/optimizer/poly_traj_optimizer.hpp`
- `traj_opt/src/poly_traj_optimizer.cpp`
- `plan_manage/src/planner_manager.cpp`
- `mm_simulator/map_generator/src/map_generator.cpp`

## Scenario Generator

`map_generator` publishes a `MarkerArray` on
`/map_generator/dynamic_obstacles`. The marker pose stores the current center,
the scale stores the diameter, and the first marker point stores the constant
velocity vector. This keeps the interface compatible with standard ROS
Noetic `visualization_msgs`.

Parameters exposed by `run_in_sim.launch`:

```text
dynamic_obs_num
dynamic_obs_radius
dynamic_obs_speed
dynamic_obs_cross_x
dynamic_obs_cross_y
dynamic_obs_z
dynamic_obs_cross_offset
dynamic_obs_spawn_delay
protected_radius
```

`dynamic_obs_spawn_delay` introduces obstacles after planning has started.
This separates two effects:

- a known future crossing tests spatio-temporal avoidance in the optimizer;
- a newly appearing obstacle tests FSM monitoring and online replanning.

`protected_radius` keeps randomly generated static boxes away only from the
configured start and task waypoints. Random boxes cover the connecting
corridor, so randomized connectivity and detours remain part of the test.

## Strong Baseline

The experiment suite includes `static_shell`, a reproducible conservative
baseline:

- optimizer samples obstacles at the current time;
- the collision shell is expanded by the maximum-speed stopping distance;
- `hard_collision_margin` and `hard_collision_margin_mani` enforce the shell
  in the final collision audit;
- the baseline uses hard collision constraints only;
- it has no recoverability manifold, joint recovery-time cost, or temporal
  repair.

This baseline accounts for the robot stopping distance explicitly and uses the
same world, robot and final audit as REMANI-R.

For the configured maximum base speed `0.7 m/s` and service deceleration
`1.4 m/s^2`, the shell is:

```text
0.7^2 / (2 * 1.4) + 0.7 * 0.01 = 0.182 m
```

The experiment rounds this to a `0.19 m` mobile-base margin.

The ordinary `safe_margin` remains the soft clearance and recoverability
buffer used by REMANI-R. The hard shell is evaluated independently in the
final audit.

The suite also includes `current_only`, which keeps the original
collision-only objective and evaluates each moving obstacle only at the
current planner time. It isolates the effect of trajectory-arrival-time
querying from the effect of the recoverability constraints.

TopAY results are reported separately in
`research/external/TOPAY_COMPARISON.md`.

## Reproduction

Static randomized matrix:

```bash
python3 tools/recovery_ablation.py \
  --modes remani braking full \
  --scenarios exp0 exp1_random \
  --seeds 30 31 32 33 34 35 36 37 38 39 \
          40 41 42 43 44 45 46 47 48 49 \
  --startup-wait 3 \
  --run-seconds 40 \
  --stop-after-second-plan \
  --output research/results/tables/recovery_large_static.csv
```

Dynamic matrix:

```bash
python3 tools/recovery_ablation.py \
  --modes remani current_only braking static_shell full \
  --scenarios exp0 exp1_random \
  --seeds 30 31 32 33 34 35 36 37 38 39 \
  --dynamic-obs-num 2 \
  --dynamic-cross-x -0.2 \
  --dynamic-cross-y 0.65 \
  --dynamic-spawn-delay 10 \
  --run-seconds 40 \
  --output research/results/legacy/recovery_dynamic.csv
```

`exp1_random` keeps the bridge task waypoints but uses randomized cuboid
obstacles. For this scenario, set `--dynamic-cross-y 1.5` because the
nominal corridor is at `y = 1.5 m`. The original fixed bridge remains a
separate stress test.

## Metrics

The raw CSV records:

- number of planned trajectories;
- wall, initialization, and optimization time;
- braking margin and recovery-time slack;
- number of dynamic obstacles visible to the planner;
- runtime replanning events;
- temporal safety repairs;
- collision notices and emergency stops.

The summary table is generated with:

```bash
python3 tools/summarize_recovery_ablation.py \
  research/results/tables/recovery_large_static.csv \
  research/results/legacy/recovery_dynamic.csv
```

## Contribution

REMANI-R contributes the following combined planning system:

1. a trajectory-arrival-time obstacle query shared by search, optimization,
   recovery auditing, and execution;
2. a recoverability manifold that includes base stopping and arm recovery
   time; and
3. path-preserving temporal repair after recoverability failure.

## Final Measured Status

Seeds `30-34`, 20 ground and 30 floating obstacles, two crossing spheres:

- mission success: `5/5`;
- emergency stops: `0`;
- safety-metric violations: `0`;
- mean planning time: `276.8 ms`;
- P95 planning time: `357.7 ms`;
- mean trajectory duration: `18.72 s`.

Piecewise time compression removed individual optimizer inflation. The final
run rejected a `69.25 s` optimizer output and compressed another path from
`51.78 -> 17.13 s` after localized recovery repair. The reported result
contains trajectories that passed the final `0.01 s` audit.
