# TopAY Comparison

## Environment

TopAY was compiled and executed in the existing `remani-noetic` container
rather than a new Docker image. A separate catkin workspace was used:

```text
/topay_ws
```

The TopAY build lives in `/topay_ws`; the REMANI build stays in `/ws`.
Missing system dependencies added approximately 42 MB. The TopAY source and
build workspace currently occupy approximately 1.1 GB.

TopAY required two upstream fixes for this environment:

- the dynamic-reconfigure file was exposed to CMake under the name that
  generates `rog_map/VizConfig.h`;
- dynamic-reconfigure and planner message targets were generated before the
  libraries that consume their headers.

## Benchmark Protocol

The official `benchmark_cuboids.launch` path was used with `rviz:=false`.
Its native benchmark configuration was adjusted to:

```text
map size              8 m x 8 m
ground obstacles      20
floating obstacles    30
random cases          100
fixed random sequence yes
planning budget       0.5 s
planner               MOMA
```

TopAY's benchmark mode generates its ESDF directly inside
`GridMap::regenerateMap()`. The 100-case planner benchmark therefore runs
without the auxiliary `pcl_render_node` sensing stream and prints its final
summary.

TopAY prints `Avg. Plan time` and `Avg. Path Length` with `ms` labels. The
code computes both from `ros::Time::toSec()`; the measured values are:

```text
successful cases       52 / 100
mean planning time     0.122170 s = 122.17 ms
mean trajectory time   8.90722 s
```

Raw log: `research/archive/raw_logs/topay_benchmark100.log`.

## REMANI Results on the Same Map Dimensions and Obstacle Counts

| Method | Scenario | Runs | Planning success | Mean wall | P95 wall | Worst braking margin | Worst recovery slack |
|---|---|---:|---:|---:|---:|---:|---:|
| REMANI | randomized cuboids | 20 | 100% | 522.1 ms | 716.3 ms | -0.2407 m | -1.1218 s |
| REMANI | randomized bridge waypoints | 20 | 100% | 396.0 ms | 642.7 ms | -0.2004 m | -1.1112 s |
| Braking baseline | randomized cuboids | 20 | 100% | 498.7 ms | 718.1 ms | -0.0644 m | -1.4997 s |
| REMANI-R | randomized cuboids | 5 | 80% | 506.6 ms | 566.0 ms | -0.0532 m | -0.0522 s |
| REMANI-R | randomized bridge waypoints | 5 | 100% | 391.6 ms | 522.3 ms | -0.0502 m | -0.0039 s |

## Interpretation

- TopAY is about 3.2-4.3 times faster than the measured REMANI variants on
  successful trials.
- TopAY solved 52% of its native random start-goal cases.
- REMANI solved the fixed task sequence on every 20-seed random map tested,
  and REMANI-R removed the measured reversibility deficits.
Measured conclusion:

```text
TopAY is a substantially faster external baseline but has a much lower
success rate in its native randomized benchmark. REMANI-R trades planning
time for task completion and whole-body recoverability.
```

