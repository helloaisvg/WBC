# Reversible Whole-Body Control

This repository implements the executable minimum loop described in the
reversibility-construction design:

- an immutable, real-time safety shell that bounds the body-level consequence
  of every command;
- an ensemble recoverability monitor trained from grounded recovery trials;
- active probing near the recoverability boundary;
- bounded online adaptation of the monitor's action-sensitive terms;
- recoverability arbitration before execution;
- shell-trigger, boundary-expansion, and irreversible-event telemetry;
- a warehouse bin-transfer simulator and side-by-side benchmark.

The planning/control rate is 100 Hz in simulation. The shell itself is a
constant-time projection and does not call the learned monitor.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

## Train, benchmark, and inspect the shell

```bash
./build/rwb_train --samples 24000 --seed 17 --output models/recoverability.model
./build/rwb_benchmark \
  --model models/recoverability.model \
  --episodes 40 \
  --seed 2026 \
  --out outputs
./build/rwb_shell_spec --out outputs/shell_spec.txt
```

The benchmark writes episode-level CSV, step-level telemetry, a machine-readable
summary, and an SVG comparison. See `docs/experiment_protocol.md` for metric
definitions and the smallest real-system transfer procedure.

## Scenario

The simulator is a loaded differential-drive mobile manipulator moving a bin
through a warehouse aisle. It models actuator response, coupled lateral load,
payload swing, body roll, disturbances, stopping clearance, and collisions.
Task irreversibility means the bin leaves its support envelope without a
recoverable return. Body irreversibility means the support margin is lost.

The scenario is intentionally narrow. What transfers is the control/data
boundary and the three measured outcomes, not a claim of open-world generality.

