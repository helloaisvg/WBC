# REMANI-R: Recoverability-Aware Spatio-Temporal Planning

## Innovation

REMANI plans collision-free whole-body trajectories. Safe motion also needs
enough clearance to stop the full robot. The reversible whole-body controller
in `reversible-wbc` checks braking distance after planning. REMANI-R moves
that recoverability condition into trajectory generation:

- Hybrid A* and RRT establish the topology.
- MINCO optimization changes base position, arm joints, and timing.
- A recoverability residual couples clearance, speed, braking deceleration,
  and control latency.
- The first two arm joints produce a normalized extension factor that inflates
  braking distance for high-inertia, extended configurations.
- Every sample also has a whole-body recovery-time slack:

```text
stop_time      = |v| / a_s + dt
joint_time_i   = minimum time to move q_i to q_i_recovery
recovery_slack = recovery_horizon - stop_time - max(joint_time)
```
- The final 0.01 s audit checks both collision and braking feasibility.

This changes the planning objective from collision-free trajectories to
reversibly safe trajectories.

## Braking Envelope

For base speed `v`, service deceleration `a_s`, and control latency `dt`:

```text
stop_distance = v^2 / (2 a_s) + |v| dt
whole_body_stop_distance = stop_distance * (1 + 0.25 * arm_extension)
available     = clearance - base_radius - safety_margin
residual      = available - whole_body_stop_distance
```

The optimizer minimizes a cubic penalty on negative residual. Its gradient
acts on both trajectory position and velocity:

- Moving toward obstacles increases the recoverability cost.
- Increasing base speed increases the recoverability cost.
- Extending the arm increases the required braking clearance.
- Tight passages force temporal deceleration instead of only spatial detour.

The final safety pass evaluates the residual every 0.01 s. A negative margin
beyond the configured tolerance rejects the trajectory.

The same 0.01 s scan evaluates recovery-time slack. The FSM safety timer also
rechecks both braking margin and recovery slack during execution, so a map
change can trigger whole-body replanning before a collision.

### Temporal Safety Repair

If optimization produces a collision-free trajectory whose only failure is
recoverability, the planner preserves its path geometry and applies an exact
polynomial time stretch:

```text
coefficient_k *= stretch^k
duration      /= stretch
```

Velocity and acceleration decrease automatically while positions remain
unchanged. The repaired trajectory is audited again, with up to three
configured stretches. The repair predicate requires collision-free geometry
with recoverability as the sole failed condition.

With a deliberately tightened `0.03 m` envelope, one `0.75x` stretch changed
the first `exp0` trajectory from `-0.0300 m`, `-0.0135 s` to
`-0.0101 m`, `+0.0442 s`. The complete plan took `361.1 ms`.

## System Integration

The implementation combines:

- REMANI Hybrid A*, layered manipulator search, MINCO trajectory, and L-BFGS.
- The stopping-distance and clearance invariant from `reversible-wbc`.
- A reusable collision-geometry cache and analytic speed/position gradient.

## REMANI-R Advancement Over REMANI

The public REMANI implementation couples hybrid whole-body search with MINCO
trajectory optimization and checks collision against a voxel/ESDF map. Its
global-map callback is designed for a static cloud, and its execution safety
check is based on that map.

REMANI-R uses the REMANI planner with a stronger planning invariant:

- collision clearance is only the first constraint;
- available clearance must also cover base stopping distance;
- arm extension changes the braking radius;
- the recovery horizon must cover base stop time and joint recovery time;
- an execution-time monitor continuously re-evaluates the invariant; and
- a recoverability-only failure can be repaired by exact polynomial time
  stretching without changing path geometry.

The current implementation additionally represents moving spherical
obstacles as a function of trajectory-arrival time. Dynamic distance is
therefore available to search, optimization, recovery auditing, and runtime
monitoring without regenerating the ESDF.

| Capability | REMANI | TopAY | DCA planner | Safe braking design | REMANI-R |
|---|---|---|---|---|---|
| Whole-body base/arm trajectory | yes | yes | yes | no | yes |
| Dynamic obstacle at arrival time | no | no | yes | no | yes |
| Speed-dependent stop envelope | no | no | no | yes | yes |
| Joint recovery-time manifold | no | no | no | no | yes |
| Runtime whole-body recovery audit | no | no | no | control layer | yes |
| Path-preserving temporal repair | no | no | no | no | yes |

The table maps the full REMANI-R capability set across the compared systems.

## Baseline Separation

The experiments use three distinct baselines:

- `remani`: collision-only objective with trajectory-arrival-time dynamic
  obstacles;
- `current_only`: collision-only objective with obstacles frozen at the
  current planner time;
- `static_shell`: collision-only objective with a maximum-speed braking
  shell.

The full method adds both the time-aware optimization query and the
whole-body recoverability manifold. Consequently, improvements over
`current_only` isolate when the obstacle is evaluated, while improvements
over `static_shell` address the cost of a time-independent conservative
envelope.

## Simulation Results

Negative margins mean the stopping distance exceeds the available clearance.
Positive margins mean the full braking envelope is satisfied.

### Dense Cuboids

| Run | Baseline margin | REMANI-R margin | Planning wall time |
|---|---:|---:|---:|
| First trajectory | -0.1898 m | -0.0337 m | 471.3 -> 383.9 ms |
| Second trajectory | -0.1695 m | -0.0642 m | 542.8 -> 552.3 ms |

The worst braking deficit decreased by `0.1561 m` and `0.1053 m`.

### Bridge

| Run | Baseline margin | REMANI-R margin | Planning wall time |
|---|---:|---:|---:|
| First trajectory | -0.1733 m | -0.0342 m | 491.6 -> 603.2 ms |
| Second trajectory | -0.1489 m | -0.0337 m | 588.0 -> 606.0 ms |

The braking deficit decreased by `0.1391 m` and `0.1152 m`. The stronger
recovery-time constraint increases planning time in this scenario, but keeps
all trajectory margins inside the configured safety tolerances.

### Whole-Body Recovery Time

The FastArmer recovery pose is configured as:

```text
[0.0, -1.2217, 1.9199, 0.0, -1.9199, 0.0]
```

| Scenario | First slack | Second slack | Wall time |
|---|---:|---:|---:|
| `exp0` | -0.0177 s | +0.3578 s | 383.9 / 552.3 ms |
| `exp1` | -0.0780 s | -0.0305 s | 603.2 / 606.0 ms |

All trajectories completed without collision rejection or emergency stop.
Negative slack remains inside the configured `0.10 s` recovery-time tolerance.

Both scenarios completed planning and execution without collision rejection
or emergency stop.

### Final Five-Seed Regression

The final implementation separates static and dynamic budgets and adds
failure-local temporal repair plus verified piecewise time compaction:

| Case | Seed count | Success | Mean wall | P95 wall | Safety violations |
|---|---:|---:|---:|---:|---:|
| Static `exp0` | 5 | 5/5 two-plan runs | 243.7 ms | 286.4 ms | 0 |
| Static `exp1_random` | 5 | 5/5 two-plan runs | 235.3 ms | 293.7 ms | 0 |
| Dynamic `exp0` | 5 | 5/5 missions | 276.8 ms | 357.7 ms | 0 |

The dynamic result also has zero emergency stops and zero runtime replans.
On seeds 30-32, final REMANI-R measured `258.4 ms` mean and `305.5 ms` P95;
original REMANI measured `214.0 ms` and `289.5 ms`. REMANI-R recorded
`0` versus `9` metric violations and `0` versus `1` emergency stop.

## Research Basis

- REMANI: real-time whole-body search and spatial-temporal optimization,
  ICRA 2024.
- Safety-distance braking for mobile manipulators:
  [Design of Maximum Stopping Deceleration of Mobile Manipulator Considering
  the Protective Separation Distance without Tip-over](https://doi.org/10.1109/isr50024.2021.9419546)
  and [Towards mobile manipulator safety standards](https://doi.org/10.1109/rose.2013.6698414).
- TopAY: arc-length-yaw parameterization reduces optimization complexity for
  differential-drive mobile manipulators,
  https://arxiv.org/abs/2507.02761.
- A reactive SDF-constrained QP framework combines learned reactivity with
  hard signed-distance safety,
  https://arxiv.org/abs/2503.23975.
- AutoMoMa uses a unified kinematic chain and GPU-parallel trajectory
  optimization for scalable data generation,
  https://arxiv.org/abs/2604.12565.
