# REMANI Planner Performance Path

## Changes

- Reuse the unexecuted suffix of the optimized trajectory for local replans
  instead of forcing a fresh search.
- Align the warm-start sampling time with the future trajectory handoff time.
- Bound L-BFGS by wall-clock time and iteration count while retaining the full trajectory safety scan.
- Reduce sparse optimization sampling from 16 to 12 points per piece and low-speed refinement from 16 to 8.
- Reuse collision-check buffers instead of allocating transforms and point vectors on every cost evaluation.
- Reuse the mobile-base collision sample buffer inside `MMConfig`; every A*
  edge previously allocated and resized roughly one hundred 3D points for each
  of its 12 time samples.
- Remove the 50 ms sleep between visualization markers and publish the complete marker array once.
- Skip frontier, global trajectory, and mesh construction when no visualization subscriber exists.
- Pass visualization payloads by const reference to remove vector and matrix copies.
- Reserve the Hybrid A* hash table and fix forward/reverse direction checks.
- Add a whole-body recoverability envelope that couples base braking distance
  with arm extension in the optimizer and final safety audit.
- Add a whole-body recovery-time manifold that couples base stopping time with
  the time required to return each joint to a configured low-inertia pose.
- Add exact polynomial time stretching as a safety repair before spawning a
  new front-end search.
- Publish a large static global cloud only a bounded number of times after a
  subscriber connects, instead of streaming it for the whole run.
- Gate mission activation on both static-cloud reception and the first ESDF
  update, eliminating plans against the uninitialized distance sentinel.
- Keep dynamic update publication at 20 Hz while avoiding dynamic ESDF
  rebuilds through analytic sphere distance queries.
- Repair recoverability or feasibility failures by lengthening only the
  polynomial suffix from the first failing sample, then re-solve the
  coefficient system so position, velocity, acceleration, and jerk continuity
  are preserved.
- Compress inflated dynamic time allocations in three safety-audited passes:
  individual pieces first, local bisection second, and a global interpolation
  between the A* seed and the safe optimized allocation.
- Use `0.05 s` coarse safety checks only while searching for a compressed time
  allocation. The selected trajectory is always rechecked at `0.01 s` before
  it can be returned.
- Separate static and dynamic optimization parameters. Static scenes keep the
  original `80 ms` budget; moving-obstacle scenes use a `60 ms` budget while
  final safety scans remain unchanged.

## Measured In Simulation

Both scenarios completed, with no collision rejection or emergency stop.

| Scenario | Original first | Original second | Optimized first | Optimized second |
|---|---:|---:|---:|---:|
| Dense cuboids, `exp0` | 526.8 ms | 643.5 ms | 417.4 ms | 541.0 ms |
| Bridge, `exp1` | 366.2 ms | 677.5 ms | 408.8 ms | 606.3 ms |

Average planning time:

- `exp0`: 585.1 ms to 479.2 ms, 18.1% lower.
- `exp1`: 521.9 ms to 507.6 ms, 2.7% lower.

Local rolling replanning:

- Initial plan: 375.5 ms.
- Warm-start replan: 157.9 ms.
- Warm-start initialization: 0.75 ms versus 177.8 ms for the initial front-end search.
- Warm-start replanning latency: 57.9% lower.

Recoverability-aware planning:

- `exp0` worst braking deficit improved from `-0.1898/-0.1695 m` to
  `-0.0337/-0.0642 m`.
- `exp1` braking margins improved from `-0.1733/-0.1489 m` to
  `-0.0342/-0.0337 m`.
- All trajectories remain inside the configured `0.07 m` braking tolerance.

Whole-body recovery time:

- `exp0` minimum recovery slack: `-0.0177 s` then `+0.3578 s`.
- `exp1` minimum recovery slack: `-0.0780 s` then `-0.0305 s`.
- All results remain inside the `0.10 s` execution tolerance.

Temporal safety repair:

- Forced `0.03 m` test: a `0.75x` stretch improved the braking margin from
  `-0.0300 m` to `-0.0101 m`.
- Recovery slack changed from `-0.0135 s` to `+0.0442 s`.
- Total planning time for the repaired trajectory was `361.1 ms`.

The measurements use the same x86_64 Ubuntu 20.04 and ROS Noetic environment on Apple Silicon.

## Final Iteration Measurements

Five-seed static regression:

| Scene | Two-plan success | Mean wall | P95 wall | Safety violations |
|---|---:|---:|---:|---:|
| `exp0` | 5/5 | 243.7 ms | 286.4 ms | 0 |
| `exp1_random` | 5/5 | 235.3 ms | 293.7 ms | 0 |

Five-seed dynamic mission:

- mission success: `5/5`;
- emergency stops: `0`;
- safety-metric violations: `0`;
- mean planning time: `276.8 ms`;
- P95 planning time: `357.7 ms`;
- mean trajectory duration: `18.72 s`;
- runtime replans: `0`.

The piecewise repair materially shortens pathological optimizer outputs.
Final-run examples include `27.6 -> 21.75 s` and `18.18 -> 15.12 s`; an earlier
same-configuration run also compressed `40.7 -> 17.5 s`. Early A* output is
the seed allocation; the final time allocation comes from the audited
compressed trajectory.

Rejected tuning:

- `lambda_heu = 1.1/1.3` did not reduce the tail safely;
- 10 dynamic A* samples caused long unsafe-looking path choices and was
  reverted to 12;
- strong quadratic temporal anchors reduced some blow-ups but destabilized
  other seeds;
- increasing obstacle or recoverability weights made the selected paths
  longer rather than faster.

## Tuning

The main knobs are in `remani_planner/plan_manage/config/remani_planner_param.yaml`:

- `optimization/max_solve_time`
- `optimization/max_iterations`
- `optimization/lbfgs_memory`
- `optimization/constrain_points_perPiece`
- `optimization/dense_sample_resolution`
