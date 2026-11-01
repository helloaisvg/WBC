# REMANI-R Research Pipeline

`research/` is the execution and results layer for the REMANI-R core in
`remani_planner/` and `mm_simulator/`.

## Pipeline

```text
study YAML
  -> mode overlays
  -> scenario arguments
  -> run_matrix.py
  -> raw trials and logs
  -> summarize.py
  -> paired_stats.py
  -> plot.py
```

## Layout

- `configs/modes`: parameter overlays for each planner mode.
- `configs/scenarios`: ROS parameter-file mapping.
- `configs/studies`: complete experiment matrices.
- `pipeline`: execution, metrics, statistics and plotting.
- `scenarios`: shared dynamic-obstacle and completion generators.
- `capture`: RViz capture and video composition.
- `runs`: immutable output for each study execution.
- `results/canonical`: selected result tables used by the current report.
- `results/tables`: ablation and baseline tables.
- `results/rejected`: rejected tuning outputs.
- `results/legacy`: earlier tables retained for comparison.
- `external`: original-REMANI, TopAY and GCDF comparison documents.
- `media`: final videos.
- `figures`: final figures and contact sheets.
- `archive`: raw videos, historical logs and large trajectories.

## Run a study

```bash
python3 research/pipeline/run_study.py \
  research/configs/studies/dynamic_fair_v1.toml
```

Each run writes:

```text
research/runs/<study>/<timestamp>/
├── manifest.json
├── raw/
│   ├── trials.csv
│   └── logs/
├── processed/
│   ├── summary.md
│   └── paired_stats.md
└── report/
    └── figures/
        └── benchmark.png
```

## Run one matrix directly

```bash
python3 research/pipeline/run_matrix.py \
  --modes remani current_only static_shell full \
  --scenarios exp0 \
  --seeds 30 31 32 33 34 \
  --dynamic-obs-num 2 \
  --run-seconds 40 \
  --output research/results/canonical/dynamic_trials.csv
```

## External original-REMANI study

`remani-bench` remains the external original-REMANI laboratory. Import only
compact comparison tables into `research/external/imported_results/`.
